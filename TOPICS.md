# `move_pkg` Topics

`move_pkg` 패키지에서 사용하는 주요 토픽 정리입니다.

## Published Topics

### `nav2_ros_controller`

| Topic | Type | 설명 |
| --- | --- | --- |
| `/move_status` | `std_msgs/msg/Bool` | 이동 명령의 최종 성공/실패 여부만 단순 발행 |
| `/move_pkg/nav/goal_received` | `std_msgs/msg/String` | 목표 명령 접수 또는 초기 거절 결과를 JSON 문자열로 발행 |
| `/move_pkg/nav/reached` | `std_msgs/msg/String` | 목표 최종 도달 성공 결과를 JSON 문자열로 발행 |
| `/move_pkg/nav/current_heading` | `std_msgs/msg/Float64` | 현재 LIGO heading 값(deg from north cw) 발행 |
| `/move_pkg/nav/current_enu` | `geometry_msgs/msg/PointStamped` | 현재 위치를 mission anchor 기준 ENU 좌표로 발행 |



## Subscribed Topics

### `nav2_ros_controller`

| Topic | Type | 설명 |
| --- | --- | --- |
| `/move_pkg/nav/goal` | `sensor_msgs/msg/NavSatFix` | 목표 위경도 입력 |
| `/ligo/global_position` | `sensor_msgs/msg/NavSatFix` | 현재 LIGO 위치 입력 |
| `/ligo/enu_heading_deg` | `std_msgs/msg/Float64` | 현재 LIGO heading 입력 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map metadata 수신 |

추가로 Nav2 action server `navigate_to_pose`를 사용합니다.

## JSON Payload Format

`/move_pkg/nav/goal_received`와 `/move_pkg/nav/reached`는 `std_msgs/msg/String` 타입이며, `data` 필드에 아래 형식의 JSON 문자열을 담습니다.

```json
{
  "command_failed": false,
  "reason": "목표 명령이 정상 접수되었습니다.",
  "start": {
    "lat": 37.4123116410167,
    "lon": 127.09331423461174
  },
  "goal": {
    "lat": 37.4115,
    "lon": 127.0931
  },
  "timestamp_unix": 1776339470.79
}
```