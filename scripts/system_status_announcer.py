#!/usr/bin/env python3

import os
import queue
import shutil
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from whill_msgs.msg import ModelCr2State


NAV2_LIFECYCLE_NODES = (
    'map_server',
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
)

# Key, startup message, lost message, recovery message
FEEDS = {
    'whill': (
        '車体の状態情報を受信しました。',
        '車体との通信が途絶えました。',
        '車体との通信が復旧しました。',
    ),
    'ouster': (
        '三次元レーザーの点群を受信しました。',
        '三次元レーザーの通信が途絶えました。',
        '三次元レーザーの通信が復旧しました。',
    ),
    'urg': (
        '障害物センサーが動作しています。',
        '障害物センサーの通信が途絶えました。',
        '障害物センサーの通信が復旧しました。',
    ),
    'odom': (
        'スラムの自己移動推定が動作しています。',
        'スラムの位置情報が途絶えました。',
        'スラムの位置情報が復旧しました。',
    ),
    'scan_ouster': (None, None, None),
}


class SpeechWorker:
    """Generate and play Japanese speech in a worker thread."""

    def __init__(self, logger, dictionary, voice_file, rate):
        self.logger = logger
        self.dictionary = dictionary
        self.voice_file = voice_file
        self.rate = rate

        self.messages = queue.Queue(maxsize=12)
        self.stop_event = threading.Event()
        self.thread = None

        self.available = (
            shutil.which('open_jtalk') is not None
            and shutil.which('aplay') is not None
            and Path(dictionary).is_dir()
            and Path(voice_file).is_file()
        )

        if not self.available:
            self.logger.error(
                'Voice output unavailable. Check Open JTalk, '
                'dictionary, voice model and aplay.'
            )
            return

        self.thread = threading.Thread(
            target=self.run,
            daemon=True,
            name='speech_worker',
        )
        self.thread.start()

    def say(self, message, valid_if=None):
        if not self.available or self.stop_event.is_set():
            return

        try:
            self.messages.put_nowait(
                (time.monotonic(), message, valid_if)
            )
        except queue.Full:
            self.logger.warning(
                f'Speech queue full. Skipped: {message}'
            )

    def run(self):
        with tempfile.TemporaryDirectory(
            prefix='whill_voice_'
        ) as directory:

            cache = {}

            while not self.stop_event.is_set():
                try:
                    item = self.messages.get(timeout=0.2)
                except queue.Empty:
                    continue

                if item is None:
                    break

                queued_at, message, valid_if = item

                try:
                    # Skip outdated messages.
                    if time.monotonic() - queued_at > 7.0:
                        continue

                    if valid_if is not None and not valid_if():
                        continue

                    wav_path = cache.get(message)

                    if wav_path is None:
                        wav_path = os.path.join(
                            directory,
                            f'voice_{len(cache)}.wav',
                        )

                        subprocess.run(
                            [
                                'open_jtalk',
                                '-x', self.dictionary,
                                '-m', self.voice_file,
                                '-r', str(self.rate),
                                '-ow', wav_path,
                            ],
                            input=message.encode('utf-8'),
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.PIPE,
                            timeout=20,
                            check=True,
                        )

                        cache[message] = wav_path

                    subprocess.run(
                        ['aplay', '-q', wav_path],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.PIPE,
                        timeout=30,
                        check=True,
                    )

                except (OSError, subprocess.SubprocessError) as error:
                    self.logger.error(
                        f'Audio playback failed: {error}'
                    )

                finally:
                    self.messages.task_done()

    def close(self):
        self.stop_event.set()

        if self.thread is not None:
            try:
                self.messages.put_nowait(None)
            except queue.Full:
                pass

            self.thread.join(timeout=1.0)


class SystemStatusAnnouncer(Node):

    def __init__(self):
        super().__init__('system_status_announcer')

        self.declare_parameter(
            'dictionary',
            '/var/lib/mecab/dic/open-jtalk/naist-jdic',
        )
        self.declare_parameter(
            'voice_file',
            '/usr/share/hts-voice/nitech-jp-atr503-m001/'
            'nitech_jp_atr503_m001.htsvoice',
        )
        self.declare_parameter('speech_rate', 1.0)
        self.declare_parameter('sensor_timeout_sec', 4.0)
        self.declare_parameter('odom_timeout_sec', 3.0)
        self.declare_parameter('localization_timeout_sec', 5.0)
        self.declare_parameter('nav2_timeout_sec', 4.0)
        self.declare_parameter('check_period_sec', 1.0)

        def param(name):
            return self.get_parameter(name).value

        self.sensor_timeout = float(param('sensor_timeout_sec'))
        self.odom_timeout = float(param('odom_timeout_sec'))
        self.localization_timeout = float(
            param('localization_timeout_sec')
        )
        self.nav2_timeout = float(param('nav2_timeout_sec'))
        self.check_period = float(param('check_period_sec'))
        speech_rate = float(param('speech_rate'))

        if min(
            self.sensor_timeout,
            self.odom_timeout,
            self.localization_timeout,
            self.nav2_timeout,
            self.check_period,
            speech_rate,
        ) <= 0.0:
            raise ValueError('Timeouts and speech rate must be positive')

        self.announcement_pub = self.create_publisher(
            String,
            '/system/announcement',
            10,
        )

        self.speech = SpeechWorker(
            self.get_logger(),
            param('dictionary'),
            param('voice_file'),
            speech_rate,
        )

        self.last_messages = {
            name: None for name in FEEDS
        }
        self.feed_states = {
            name: 'waiting' for name in FEEDS
        }

        self.whill_power_ok = False
        self.whill_power_announced = None

        self.last_status = ''
        self.last_status_time = None
        self.last_accept = None

        self.ever_localized = False
        self.localized = False
        self.asked_initial_pose = False
        self.nav_active = False
        self.system_ready = False

        # -------------------------------------------------
        # Subscriptions
        # -------------------------------------------------

        self.monitor_subscriptions = [
            self.create_subscription(
                ModelCr2State,
                '/whill/states/model_cr2',
                self.on_whill,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                PointCloud2,
                '/ouster/points',
                lambda msg: self.mark('ouster'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                LaserScan,
                '/scan_urg',
                lambda msg: self.mark('urg'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                '/odom_nav',
                lambda msg: self.mark('odom'),
                10,
            ),
            self.create_subscription(
                LaserScan,
                '/scan_ouster',
                lambda msg: self.mark('scan_ouster'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                String,
                '/localization/status',
                self.on_localization_status,
                10,
            ),
        ]

        # -------------------------------------------------
        # Nav2 lifecycle monitoring
        # -------------------------------------------------

        self.lifecycle_clients = {
            name: self.create_client(
                GetState,
                f'/{name}/get_state',
            )
            for name in NAV2_LIFECYCLE_NODES
        }

        self.active_updates = {
            name: None for name in NAV2_LIFECYCLE_NODES
        }
        self.pending_requests = {}
        self.pending_since = {}

        # A steady clock remains usable when /clock is paused.
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.timer = self.create_timer(
            self.check_period,
            self.check_status,
            clock=steady_clock,
        )

        self.announce('システムを起動します。')

    # -----------------------------------------------------
    # Announcements
    # -----------------------------------------------------

    def announce(self, message, valid_if=None):
        self.get_logger().info(f'Announcement: {message}')
        self.announcement_pub.publish(String(data=message))
        self.speech.say(message, valid_if)

    def mark(self, name):
        self.last_messages[name] = time.monotonic()

    @staticmethod
    def is_fresh(stamp, now, timeout):
        return (
            stamp is not None
            and 0 <= now - stamp <= timeout
        )

    # -----------------------------------------------------
    # WHILL
    # -----------------------------------------------------

    def on_whill(self, msg):
        self.mark('whill')

        self.whill_power_ok = (
            bool(msg.power_on)
            and int(msg.error) == 0
        )

    # -----------------------------------------------------
    # GICP localization
    # -----------------------------------------------------

    def on_localization_status(self, msg):
        status = msg.data.strip()

        self.last_status = status
        self.last_status_time = time.monotonic()

        if status.startswith('ACCEPT'):
            self.last_accept = time.monotonic()

        elif (
            status == 'INITIALPOSE_ACCEPTED_WAIT_REGISTRATION'
            or status.startswith('WAIT_INITIALPOSE')
        ):
            self.last_accept = None

    # -----------------------------------------------------
    # Nav2 lifecycle
    # -----------------------------------------------------

    def on_lifecycle_state(self, name, future):
        if self.pending_requests.get(name) is not future:
            return

        self.pending_requests.pop(name, None)
        self.pending_since.pop(name, None)

        try:
            result = future.result()

            if (
                result is not None
                and result.current_state.id
                == State.PRIMARY_STATE_ACTIVE
            ):
                self.active_updates[name] = time.monotonic()
            else:
                self.active_updates[name] = None

        except Exception as error:
            self.active_updates[name] = None
            self.get_logger().debug(
                f'Lifecycle query failed for {name}: {error}'
            )

    def poll_lifecycle(self, now):
        for name, client in self.lifecycle_clients.items():

            pending = self.pending_requests.get(name)

            if pending is not None:
                if (
                    now - self.pending_since[name]
                    <= self.nav2_timeout
                ):
                    continue

                self.pending_requests.pop(name, None)
                self.pending_since.pop(name, None)
                self.active_updates[name] = None
                pending.cancel()

            if not client.service_is_ready():
                self.active_updates[name] = None
                continue

            try:
                future = client.call_async(GetState.Request())

                self.pending_requests[name] = future
                self.pending_since[name] = now

                future.add_done_callback(
                    lambda f, node_name=name:
                    self.on_lifecycle_state(node_name, f)
                )

            except Exception as error:
                self.active_updates[name] = None
                self.get_logger().debug(
                    f'Lifecycle request failed: {error}'
                )

    # -----------------------------------------------------
    # Periodic monitoring
    # -----------------------------------------------------

    def check_status(self):
        now = time.monotonic()

        self.poll_lifecycle(now)

        # Sensor and odometry freshness.
        for name, messages in FEEDS.items():
            started, lost, recovered = messages

            timeout = (
                self.odom_timeout
                if name == 'odom'
                else self.sensor_timeout
            )

            fresh = self.is_fresh(
                self.last_messages[name],
                now,
                timeout,
            )

            previous = self.feed_states[name]

            if fresh:
                state = 'ok'
            elif previous in ('ok', 'lost'):
                state = 'lost'
            else:
                state = 'waiting'

            if previous == state:
                continue

            self.feed_states[name] = state

            if started is None:
                continue

            if previous == 'waiting' and state == 'ok':
                self.announce(started)

            elif previous == 'ok' and state == 'lost':
                self.announce(lost)

            elif previous == 'lost' and state == 'ok':
                self.announce(recovered)

        # WHILL power and error state.
        if self.feed_states['whill'] == 'ok':
            if self.whill_power_ok != self.whill_power_announced:

                if self.whill_power_ok:
                    self.announce(
                        '車体の電源と状態を確認しました。'
                    )
                else:
                    self.announce(
                        '車体の電源またはエラー状態を確認してください。'
                    )

                self.whill_power_announced = self.whill_power_ok

        # Initial pose request.
        if (
            self.last_status.startswith('WAIT_INITIALPOSE')
            and self.feed_states['odom'] == 'ok'
        ):
            if not self.asked_initial_pose:
                self.announce(
                    '初期位置を指定してください。'
                )
                self.asked_initial_pose = True

        elif (
            self.last_status
            == 'INITIALPOSE_ACCEPTED_WAIT_REGISTRATION'
            or self.last_status.startswith('ACCEPT')
        ):
            self.asked_initial_pose = False

        # GICP localization freshness.
        localized_now = (
            self.is_fresh(
                self.last_accept,
                now,
                self.localization_timeout,
            )
            and self.is_fresh(
                self.last_status_time,
                now,
                self.localization_timeout,
            )
        )

        if localized_now and not self.localized:
            if self.ever_localized:
                self.announce(
                    '自己位置推定が復旧しました。'
                )
            else:
                self.announce(
                    '自己位置推定が完了しました。'
                )

            self.ever_localized = True

        elif self.localized and not localized_now:
            self.announce(
                '自己位置推定が失われました。'
            )

        self.localized = localized_now

        # Nav2 lifecycle state.
        nav_now = all(
            self.is_fresh(
                self.active_updates[name],
                now,
                self.nav2_timeout,
            )
            for name in NAV2_LIFECYCLE_NODES
        )

        if nav_now and not self.nav_active:
            self.announce(
                'ナビゲーションが起動しました。'
            )

        elif self.nav_active and not nav_now:
            self.announce(
                'ナビゲーションの状態を確認してください。'
            )

        self.nav_active = nav_now

        # Combined system readiness.
        ready = (
            self.whill_power_ok
            and all(
                state == 'ok'
                for state in self.feed_states.values()
            )
            and self.localized
            and self.nav_active
        )

        was_ready = self.system_ready
        self.system_ready = ready

        if ready and not was_ready:
            self.announce(
                'システムの準備が完了しました。',
                valid_if=lambda: self.system_ready,
            )

    def close(self):
        self.speech.close()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = SystemStatusAnnouncer()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.close()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()