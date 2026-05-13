# `move_pkg` Topics

`move_pkg` 패키지에서 사용하는 주요 토픽 정리입니다.

## Published Topics

### `nav2_ros_controller`

| Topic | Type | 설명 |
| --- | --- | --- |
| `/move_status` | `std_msgs/msg/Bool` | 이동 명령의 최종 성공/실패 여부만 단순 발행 |
| `/move_pkg/nav/goal_received` | `move_pkg/msg/GoalWithHeading` | 수신한 목표(위경도·heading)와 `stamp`로 접수/거절 시점을 알림 (`/move_status`로 성공·실패 확인) |
| `/move_pkg/nav/reached` | `move_pkg/msg/GoalWithHeading` | 최종 도달·heading 정렬 완료 시 목표 위경도·heading과 `stamp` 발행 |
| `/move_pkg/nav/current_heading` | `std_msgs/msg/Float64` | 현재 LIGO heading 값(deg from north cw) 발행 |
| `/move_pkg/nav/current_enu` | `geometry_msgs/msg/PointStamped` | 현재 위치를 mission anchor 기준 ENU 좌표로 발행 |



## Subscribed Topics

### `nav2_ros_controller`

| Topic | Type | 설명 |
| --- | --- | --- |
| `/move_pkg/nav/goal_with_heading` | `move_pkg/msg/GoalWithHeading` | 목표 위경도 + 목표 heading 입력 |
| `/ligo/global_position` | `sensor_msgs/msg/NavSatFix` | 현재 LIGO 위치 입력 |
| `/ligo/enu_heading_deg` | `std_msgs/msg/Float64` | 현재 LIGO heading 입력 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map metadata 수신 |

추가로 Nav2 action server `navigate_to_pose`를 사용합니다.

접수 거절·Nav2 실패 등 상세 사유는 노드 로그와 `/move_status`를 참고합니다. `goal_received` / `reached`의 `heading_deg_from_north_cw`가 NaN이면 입력에 목표 heading이 없었던 경우입니다.

## Custom Message

`move_pkg/msg/GoalWithHeading`:

```text
builtin_interfaces/Time stamp
float64 latitude
float64 longitude
float64 heading_deg_from_north_cw
```