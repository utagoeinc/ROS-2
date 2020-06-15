# Examples
Here are PySphero example scripts.
These are the overview of each script.

| Name | Overview |
| --- | --- |
| drive_example.py | Drive random direction when 'd' is input |
| drive_with_heading.py | Drive random direction for 2 sec five times |
| find_toy.py | Find Sphero and show its MAC address |
| police.py | Blink LED red and blue like police car |
| sensors.py | Show quaternion information |
| wake.py | Wake up Sphero |

# Class Information of PySphero
## Class: Sphero
| Method | Package | Return Type |
| --- | --- | --- |
| system_info | pysphero.device_api | SystemInfo |
| power | pysphero.device_api | Power |
| driving | pysphero.driving | Driving |
| api_processor | pysphero.device_api | ApiProcessor |
| user_io | pysphero.device_api | UserIO |
| sensor | pysphero.device_api | Sensor |
| animatronics | pysphero.device_api | Animatronics |

## Class: SystemInfo
source: `pysphero/device_api/system_info.py`

| Method | Return Type | Overview |
| --- | --- | --- |
| get_main_application_version | Version | Get version of toy's firmware |
| get_bootloader_version | Version | Get version of toy's bootloader |
| get_mac_address | str | Get toy's mac address as "aa:bb:cc:dd:ee:ff" |
| get_nordic_temperature | int | Return celsius temperature |
| get_status_id | int | ??? |
| get_sku | str | ??? |

### Class: Version
```python
class Version(NamedTuple):
    major: int
    minor: int
    revision: int
```

## Class: Power
source: `pysphero/device_api/power.py`

| Method | Return Type | Overview |
| --- | --- | --- |
| enter_deep_sleep | - | Send shutdown command to toy |
| enter_soft_sleep | - | Send sleep command to toy |
| get_battery_voltage | float | Get battery voltage |
| wake | None | Wake up toy |
| get_battery_state | BatteryVoltageStates | Get battery state without known volatage constants |
| battery_state_changed | ChagerStates | Get charging status information |

```python
class BatteryVoltageStates(UnknownEnumMixing, Enum):
    ok = 0x01
    low = 0x02
    critical = 0x03


class ChargerStates(UnknownEnumMixing, Enum):
    not_charging = 0x01
    charging = 0x02
    charged = 0x03
```

## Class: Driving
source: `pysphero/driving.py`

| Method | Argument | Function |
| --- | --- | --- |
| drive_with_heading | speed(int: 0 to 255), headin(int: 0 to 360), direction(Direction) | Drive heading to `heading` with speed `speed` |
| set_stabilization | stabilization_index: StabilizationIndex | ??? |
| raw_motor | left_speed(int: 0 to 255), left_direction(DirectionRowMotor), right_speed(int: 0 to 255), right_direction(DirectionRawMotor) | Control of each motor separately. (Note it works strange) |
| reset_yaw | None | Reset of toy's yaw |
| tank_drive | left_speed(int), right_speed(int), direction(TankDriveDirection) | For Sphero RVR? |

## Class: ApiProcessor
source: `pysphero/device_api/api_processor.py`

| Method | Overview |
| --- | --- |
| echo | Send ping |

## Class: UserIO
source: `pysphero/device_api/user_io.py`

| Method | Argument | Function |
| --- | --- | --- |
| set_all_leds_8_bit_mask | front_color(Color), back_color(Color) | Set LEDs color (for only Sphero BOLT) |
| set_led_matrix_one_color | color(Color) | Set matrix one color |
| set_led_matrix_pixel | pixel(Pixel), color(Color) | Put into pixel the color |
| set_led_matrix_single_character | symbol(str), color(Color) | Print symbol on matrix |
| set_led_matrix_text_scrolling | string(str: max 6 symbols), color(Color), speed(int: max 30), repeat(bool) | Print text on matrix |
| set_led_matrix_text_scrolling_notify | None | ??? |
| set_led_matrix_frame_rotation | rotation(FrameRotation) | ??? |

## Class: Sensor
source: `pysphero/device_api/sensor.py`

*Mandatory
| Method | Argument | Return Type | Overview |
| --- | --- | --- | --- |
| set_notify | *callback(Callback function), *sensors(Sensors class), interval(int), count(int), timeout(float) | None | Set notifications from sensors |
| cancel_notify_sensors | None | None | Cancel notifications from sensors |
| get_sensor_streaming_mask | None | Tuple | ??? |
| get_ambient_light_sensor_value | None | float | ??? |
| magnetometer_calibrate_to_north | None | None | ??? |

### Alsl Sensor Class
| Class | Attribute |
| --- | --- |
| Quaternion | x, y, z, w |
| Attituede | pitch, roll, yaw |
| Accelerometer | x, y, z |
| AccelOne | accel_one |
| Locator | x, y |
| Velocity | x, y |
| Speed | speed |
| CoreTime | core_time |
| Gyroscope | x, y, z |
| AmbientLight | ambient_light |


### How to Use Example
[sensors.py](sensors.py)
```python
from time import sleep
from typing import Dict

from pysphero.core import Sphero
from pysphero.device_api.sensor import CoreTime, Quaternion


def notify_callback(data: Dict):
    info = ", ".join("{:1.2f}".format(data.get(param)) for param in Quaternion)
    print(f"[{data.get(CoreTime.core_time):1.2f}] Quaternion (x, y, z, w): {info}")
    print("=" * 60)


def main():
    mac_address = "d6:bc:6a:05:79:b6"
    with Sphero(mac_address=mac_address) as sphero:
        sphero.power.wake()
        sphero.sensor.set_notify(notify_callback, CoreTime, Quaternion)
        sleep(2)
        sphero.sensor.cancel_notify_sensors()
        sphero.power.enter_soft_sleep()


if __name__ == "__main__":
    main()

```

## Class: Animatronics
source: `pysphero/device_api/animatronics.py`

*Mandatory
| Method | Argument | Return Type |
| --- | --- | --- |
| play_animation | *animation_id(int), target_id | None |
| play_animation_and_wait | *animation_id(int), target_id, timeout(float) | None |
| perform_leg_action | *leg_action(R2LegAction) | None |
| set_head_position | *position(float) | None |
| get_head_position | None | float |
| set_leg_position | *position(float) | None |
| get_leg_position | None | float |
| get_leg_action | None | R2LegAction |
| stop_animation | None | None |
| get_trophy_mode_enabled | None | bool |
