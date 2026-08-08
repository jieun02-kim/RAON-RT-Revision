# Indy7 로봇팔 제어 코드 (`App/Indy7`) 전체 분석

> `App/Indy7`에서 `make`로 생성되는 실행파일(`Indy7Ctrl.out`)의 빌드 의존 관계와, 사용되는 각 소스 파일의 기능을 정리한 문서.

## 1. 한눈에 보는 결론

`make`를 실행하면 [Makefile](App/Indy7/Makefile)이 **18개의 `.cpp` 파일**을 컴파일·링크해서 **`bin/Indy7Ctrl.out`** 하나를 만듭니다. 이 실행파일은 **EtherCAT + 실시간(RT) POSIX 기반의 6축 협동로봇 토크 제어기**입니다.

소스는 3개 디렉토리에 흩어져 있습니다.

- **`App/Indy7/`** (현재 폴더) — 애플리케이션·제어기·비전 = "이 로봇만의 로직"
- **`CRobot/`** — 로봇/축/센서 추상화 계층 = "하드웨어 드라이버"
- **`Global/`** — 설정 파서·통신·전역 정의 = "공용 유틸"

추가로 저장소에 없는 **외부 선설치 라이브러리**에 의존합니다: NRMK EtherCAT 마스터(`/opt/emaster_app`), 실시간 라이브러리(`/opt/rt_posix`), IgH EtherLab(`/opt/etherlab`), 동역학 라이브러리 RBDL, 비전 라이브러리 ViSP/RealSense, 선형대수 Eigen3.

---

## 2. 빌드 흐름 (Makefile 해부)

[Makefile](App/Indy7/Makefile#L55-L73)이 컴파일하는 소스 목록 (`obj/*.o` → 링크 → `bin/Indy7Ctrl.out`):

| # | 소스 파일 | 위치 | 계층 |
|---|-----------|------|------|
| 1 | `ConfigParser.cpp` | Global | 공용 유틸 |
| 2 | `CRC16.cpp` | Global/Comm | 공용 유틸 |
| 3 | `Socket.cpp` | Global/Comm | 공용 유틸 |
| 4 | `Axis.cpp` | CRobot | 축(모터) 추상 |
| 5 | `AxisCIA402.cpp` | CRobot | CiA402 드라이브 |
| 6 | `AxisNRMKCore.cpp` | CRobot | NRMK 축 |
| 7 | `Sensor.cpp` | CRobot | 센서 추상 |
| 8 | `SensorFT.cpp` | CRobot | 힘/토크 센서 |
| 9 | `SensorNRMKEndTool.cpp` | CRobot | NRMK 엔드툴 |
| 10 | `ConfigRobot.cpp` | CRobot | 설정 로더 |
| 11 | `ExtInterface.cpp` | CRobot | 외부 TCP 통신 |
| 12 | `Robot.cpp` | CRobot | 로봇 베이스 |
| 13 | `Controller.cpp` | App/Indy7 | 제어기 베이스 |
| 14 | `FullDynControllerRT.cpp` | App/Indy7 | **동역학 제어기(핵심)** |
| 15 | `CalibCapture.cpp` | App/Indy7 | 핸드-아이 캘리브 |
| 16 | `VisualServo.cpp` | App/Indy7 | 비주얼 서보잉 |
| 17 | `Indy7Ctrl.cpp` | App/Indy7 | **메인 제어 클래스(핵심)** |
| 18 | `main.cpp` | App/Indy7 | 진입점 |

**컴파일 옵션**: `g++ -O2 -std=gnu++17` ([Makefile:31](App/Indy7/Makefile#L31))

**링크 라이브러리** ([Makefile:34-53](App/Indy7/Makefile#L34-L53)):
`-lrt -lpthread`(실시간/스레드) · `-lEMasterApp`(NRMK EtherCAT) · `-lrtposix`(RT) · `-lrbdl -lrbdl_urdfreader`(동역학) · `-lvisp_* -lrealsense2`(비전)

> ⚠️ **주의**: 빌드는 `/opt/emaster_app`, `/opt/rt_posix`, `/opt/etherlab`, `/home/raimlab/visp/...` 등 **하드코딩된 절대경로**를 참조합니다. 이 경로에 라이브러리가 설치돼 있지 않으면 컴파일/링크가 실패합니다.

---

## 3. 전체 아키텍처 (실행 시 구조)

```
                         main.cpp  (진입점)
                            │  설정 읽고 CRobotIndy7 생성 → Init()
                            ▼
   ┌──────────────────  CRobotIndy7 (Indy7Ctrl.cpp)  ──────────────────┐
   │   CRobot 상속. EtherCAT·RT태스크·제어기·비전을 통합 관리           │
   │                                                                   │
   │   6개 실시간 태스크 (INDY7.cfg에서 주기/우선순위 정의)            │
   │   ├ proc_main_control     (1ms, P97) ─ 제어 루프의 심장          │
   │   ├ proc_ethercat_control (1ms, P95) ─ 슬레이브 I/O              │
   │   ├ proc_keyboard_control        ─ 키 입력(모드 전환)           │
   │   ├ proc_terminal_output  (1s)   ─ 상태 출력                    │
   │   ├ proc_logger                  ─ CSV 궤적 저장                │
   │   └ proc_visual_servo            ─ 카메라(비RT)                 │
   └───────┬───────────────┬───────────────┬───────────────┬─────────┘
           ▼               ▼               ▼               ▼
  FullDynControllerRT   CAxisNRMKCore   CSensorNRMKEndTool  CExtInterface
  (토크 계산: 중력보상   (모터 토크 명령  (F/T센서·LED·       (TCP 7420 포트
   /CTC/IK, RBDL)        /엔코더 읽기)    그리퍼)            원격 명령/상태)
           │               │               │               │
           └─ Eigen/RBDL    └─ CiA402/EtherCAT (libEMasterApp) ─┘
                                            │
                            Global: ConfigParser · Socket · CRC16 · Defines.h
```

핵심 제어 데이터 흐름은 매 1ms마다:
**축에서 q, q̇ 읽기 → `FullDynControllerRT::Update()`로 토크 τ 계산 → 각 축에 `MoveTorque(τ)` 전송** 입니다.

---

## 4. 파일별 기능 정리

### 4-A. 진입점 & 설정

#### [main.cpp](App/Indy7/main.cpp) (101줄)

프로그램 시작점. 흐름:

1. `-f` 옵션으로 설정파일 경로 받기 (기본값 `ELMO.cfg` — **이 로봇은 `-f INDY7.cfg`로 실행해야 함**)
2. `CConfigRobot` 생성 → `ReadConfiguration()` → 시뮬레이션 모드 여부 결정
3. `CRobotIndy7` 생성, `SIGTERM/SIGINT` 핸들러 등록
4. `mlockall()` — 메모리를 스왑 못 하게 잠금(실시간성 확보)
5. `g_cRobot->Init(bSimMode)` — 실제 초기화(아래 전부)
6. `while(!CheckStopTask()) usleep(1000)` — 메인 스레드는 대기, 실제 일은 RT 태스크들이 함
7. 시그널 수신 시 `DeInit()`

#### [INDY7.cfg](App/Indy7/INDY7.cfg) (설정값)

실행파일의 동작을 결정하는 INI 파일. 6축 관절 게인(KP/KD), 6개 RT 태스크 주기/우선순위, EtherCAT 마스터(슬레이브 7개=관절6+엔드툴1, DC 1ms), 축별 기어비·토크상수·엔코더·홈잉·리미트, 외부 인터페이스 포트(7420), URDF 경로 등.

> ⚠️ `URDF_PATH=/home/raimlab/RAON-RT/App/Indy7/indy7.urdf` 처럼 경로가 **다른 PC 기준**으로 박혀 있습니다. 본인 환경(`/home/jaehyeon/RAON-RT-Revision-main/...`)에 맞게 수정 필요.

---

### 4-B. 애플리케이션·제어 계층 (`App/Indy7/`)

#### [Indy7Ctrl.cpp](App/Indy7/Indy7Ctrl.cpp) / [.h](App/Indy7/Indy7Ctrl.h) (1358줄) — **메인 제어 클래스 `CRobotIndy7`**

`CRobot`을 상속해 이 로봇 고유의 모든 것을 조립·구동합니다.

- **초기화**: `InitController()`(RBDL 제어기 생성·URDF 로드·게인 설정·500µs 데드라인), `InitEtherCAT()`(마스터/슬레이브 7개 생성, cfg값 적용), `Init()`(제어기→비전→부모 Init 순)
- **제어기 연동**: `EnableController()`, `SetControllerMode()`(중력보상/풀다이내믹/CTC/적응), `SetControllerGains()`
- **6개 RT 태스크 함수**(아래 표) — cfg에 정의된 주기/우선순위로 `CRobot::InitRTTasks()`가 생성·기동

| 태스크 | 주기 | 하는 일 |
|--------|------|---------|
| `proc_main_control` | 1ms | 키입력 처리 → 축에서 q/q̇/τ 읽기 → **제어기 Update()로 토크 계산 → 각 축 MoveTorque()** → 비전서보/ISO테스트/사각형모션 상태머신 → LED |
| `proc_ethercat_control` | 1ms | `ReadSlaves()` → 외부I/F 갱신 → `WriteSlaves()`, 마스터/슬레이브 OP 상태 점검, 주기 모니터링 |
| `proc_keyboard_control` | - | `r`제어토글 `g/f/c`모드 `i/m`IK `v`비전 `l`로깅 `q`ISO테스트 `s`포즈저장 등 |
| `proc_terminal_output` | 1s | 상태 출력(대부분 주석처리됨) |
| `proc_logger` | - | 트리거 시 24초간 링버퍼 수집 → `rt_log_results/*.csv` 저장 |
| `proc_visual_servo` | - | 비RT(SCHED_OTHER)로 강등해 카메라 블로킹 격리, `VisualServo::Loop()` 호출 |

- **부가 기능**: ISO 9283 정밀도 테스트(5점×10사이클, AP/RP 계산 후 CSV), TCP 궤적 로깅, 포즈 저장, 사각형 모션.

#### [Controller.cpp](App/Indy7/Controller.cpp) / [.h](App/Indy7/Controller.h) (164줄) — **제어기 베이스 `CController`**

모든 제어기의 추상 부모. 순수가상 `Init/Update/Reset/SetGains` 정의. 공통 기능으로 **출력 토크 포화(`SaturateOutput`/`Clamp`)**, 입력 NaN/Inf·크기 검증(`ValidateInputs`), 통계(`UpdateStats`), 출력 리미트(기본 ±1000 Nm), 샘플타임(기본 1ms) 제공.

#### [FullDynControllerRT.cpp](App/Indy7/FullDynControllerRT.cpp) / [.h](App/Indy7/FullDynControllerRT.h) (1463줄) — **동역학 제어기 `CControllerFullDynamicsRT` (가장 중요)**

RBDL + URDF로 로봇 동역학을 계산해 토크를 만드는 핵심. `Update()`가 모드별로 분기합니다 ([:224](App/Indy7/FullDynControllerRT.cpp#L224)):

| 제어 모드 | 함수 | 제어 법칙 |
|-----------|------|-----------|
| 중력보상 | `ComputeGravityCompensation` [:270](App/Indy7/FullDynControllerRT.cpp#L270) | **τ = g(q)** — 중력만 상쇄(수동 조작 가능) |
| 풀다이내믹스 | `ComputeFullDynamics` [:284](App/Indy7/FullDynControllerRT.cpp#L284) | **τ = M(q)q̈_ref + h(q,q̇) + (Kp·e + Kd·ė)** |
| 계산토크(CTC) | `ComputeComputedTorque` [:323](App/Indy7/FullDynControllerRT.cpp#L323) | **τ = M(q)[q̈_ref + Kp·e + Kd·ė] + h(q,q̇)** |
| 자코비안 IK | `ComputeJacobianBasedInverseKinematics` [:1211](App/Indy7/FullDynControllerRT.cpp#L1211) | 6D 자코비안 + DLS 의사역행렬로 q̇_ref → CTC |
| 6DOF IK | `ComputeInverseKinematics_6dof` [:1136](App/Indy7/FullDynControllerRT.cpp#L1136) | RBDL 전체구속 IK → 궤적 → CTC (레거시) |
| 적응제어 | (선언만) | **미구현** — 호출 시 중력보상으로 폴백 |

- **동역학 계산**: RBDL의 `CompositeRigidBodyAlgorithm`(질량행렬 M), `NonlinearEffects`(h=코리올리+중력), `InverseDynamics`(중력 g) 사용.
- **기구학**: `ComputeTcpFK`(순기구학, TCP 위치/자세), `GetCurrentPose`, `SetTargetPose`(RBDL `InverseKinematics` 전체구속), `SetTargetPosePositionOnly`(위치만).
- **궤적**: `UpdateTrajectory` [:656](App/Indy7/FullDynControllerRT.cpp#L656) — **5차 다항식**(10s³−15s⁴+6s⁵)으로 부드러운 관절 보간.
- **비주얼 서보**: `SetTargetPose_Jacobian` [:702](App/Indy7/FullDynControllerRT.cpp#L702) — 매 사이클 목표 추종, manipulability 기반 적응형 DLS 댐핑 + z축 정렬 + 속도/가속도 클램핑.
- **안전장치**: 관절 속도 상한(0.5~0.6 rad/s), 가속도 한계(2 rad/s²), `MAX_REF_LEAD`로 레퍼런스가 현재값을 앞서지 못하게 제한(오버슈트 방지).
- **실시간 최적화**: `InitRTMemoryOptimizations`(메모리 prefault), Eigen malloc 비활성, SSE flush-to-zero, `read_timer()`로 매 사이클 연산시간 측정 후 500µs 데드라인 위반 카운트.
- **검증**: `RunISOCubeIKValidation` [:1340](App/Indy7/FullDynControllerRT.cpp#L1340) — ISO 9283 큐브 5점에 대해 cold/warm IK 정확도·시간 측정 → CSV → python 플롯 자동 실행.

#### [VisualServo.cpp](App/Indy7/VisualServo.cpp) / [.h](App/Indy7/VisualServo.h) (234줄) — **비주얼 서보잉**

RealSense 카메라로 **AprilTag(36h11)**를 검출해 로봇이 추종할 목표 TCP 포즈를 만듭니다. `Init()`에서 `camera.xml`(내부파라미터)·`rPc.yaml`(손-눈 변환) 로드, RealSense 연결 확인. `Loop()`(비RT 스레드)에서 컬러프레임 획득→그레이변환→태그검출→`goal = rMc·cMo·Toffset` 계산→**지수이동평균(α=0.15) 필터**→이중버퍼에 저장. 태그 분실/특이점 상태머신 포함. ViSP/RealSense 타입은 **PIMPL**로 헤더에서 숨김.

#### [CalibCapture.cpp](App/Indy7/CalibCapture.cpp) / [.h](App/Indy7/CalibCapture.h) (44줄) — **핸드-아이 캘리브레이션 캡처**

키 입력 시 현재 TCP 포즈를 `vpHomogeneousMatrix`→`vpPoseVector`로 변환해 `calib_data/pose_rPe_N.yaml`로 저장. 손-눈 캘리브레이션용 데이터셋 수집기.

#### [DataRecorder.h](App/Indy7/DataRecorder.h) — **실시간 로깅 버퍼**

`ST_LOG_ENTRY`(타임스탬프, q/q_ref/q̇/τ 6축, TCP 포즈, 목표 포즈, 제어모드) 구조체와 **락-프리 링버퍼**(`RingBuffer<T,65536>`, atomic head/tail). RT 태스크가 `push`(가득 차면 드롭, 블로킹 없음), 로거 태스크가 `pop`. 1kHz×10분 분량.

---

### 4-C. 로봇 추상화 계층 (`CRobot/`)

#### [Robot.cpp](CRobot/Robot.cpp) / [.h](CRobot/Robot.h) (257줄) — **로봇 베이스 `CRobot`**

`CRobotIndy7`의 부모. 축(`m_vAxis`)·RT태스크·EtherCAT마스터·외부I/F·설정을 보유. `Init(sim)`이 시뮬레이션이 아니면 **외부I/F→EtherCAT→RT태스크** 순으로 초기화. `InitRTTasks()`가 cfg의 태스크 설정대로 `create_rt_task`/`set_task_period`/`start_task` 호출. `AddAxis/AddTaskFunction/AddHand`로 구성요소 등록. `DeInit()` 시 각 축의 현재 엔코더 위치를 `POS_BEFORE_EXIT`로 cfg에 저장(다음 기동 시 홈 기준).

#### [Axis.cpp](CRobot/Axis.cpp) / [.h](CRobot/Axis.h) (722줄) — **축(모터) 추상 베이스 `CAxis`**

모터 1개를 표현하는 방대한 추상 클래스. 상속: **`CAxis` ← `CAxisCIA402` ← `CAxisNRMKCore`**. 제공 기능:

- **명령**: `ServoOn/Off`, `DoHoming`, `MoveAxis/MoveHome`, `MoveVelocity`, `MoveTorque`, `StopAxis/EmgStopAxis`
- **상태 읽기**: `GetCurrentPos/Vel/Tor`(물리단위), `GetCurrentRawPos/...`(엔코더 카운트)
- **단위 변환**: `ConvertRadMM2Res`/`ConvertRes2RadMM`(엔코더↔rad), `ConvertTor2Res`/`ConvertRes2Tor`(토크↔카운트), `ConvertCur2Tor`(전류↔토크) — 기어비·엔코더해상도·토크상수 기반
- **리미트/안전**: `SetPositionLimits`·`SetVelocityLimits`·`SetTorqueLimits` 등과 `IsAllowablePosition/Velocity/...` 검사
- **상태머신**(`eAxisState`): Emergency/Error/Init/Idle/Homing/Running/Stopped

#### [AxisCIA402.cpp](CRobot/AxisCIA402.cpp) / [.h](CRobot/AxisCIA402.h) (563줄) — **CiA402 드라이브 계층 `CAxisCIA402`**

산업표준 **CiA402 모션 프로파일**을 EtherCAT 슬레이브(`CSlaveCIA402`, libEMasterApp)로 구현. 드라이브모드(CSP/CSV/CST 등) 전환(`ChangeDriveMode`), `ServoOn(driveMode)`, StatusWord/ControlWord 접근, 목표 위치/속도/토크 설정(`SetTargetPos/Tor`), DC(분산클럭) 정보 설정. `OnSlaveStatusChanged` 콜백으로 슬레이브 상태→축 상태 동기화.

#### [AxisNRMKCore.cpp](CRobot/AxisNRMKCore.cpp) / [.h](CRobot/AxisNRMKCore.h) (125줄) — **NRMK 코어 축 `CAxisNRMKCore`**

INDY7 실제 관절에 쓰이는 최종 클래스(`Indy7Ctrl`이 사용). `MoveTorque(τ)` [:36](CRobot/AxisNRMKCore.cpp#L36)가 **토크→모터전류→엔코더카운트**로 변환:
`전류 = τ / 기어비 / 토크상수`, `목표 = 전류 × 전류비 × 방향`. 그리고 `OnSlaveStatusChanged` [:57](CRobot/AxisNRMKCore.cpp#L57)에서 서보 상태(eServoOff/Idle/Homing/Running/Fault…)를 축 상태머신으로 매핑하고, 최초 기동 시 홈 위치를 설정.

#### [Sensor.cpp](CRobot/Sensor.cpp) / [.h](CRobot/Sensor.h) (245줄) — **센서 추상 베이스 `CSensor`**

모든 센서의 부모. 상속: **`CSensor` ← `CSensorFT` ← `CSensorNRMKEndTool`**. 수명주기(`Init/Start/Stop/Reset`, 순수가상), 상태머신(11종), 리미트/과부하 검사(`CheckLimits`, `CheckOverload`), 캘리브레이션 프레임워크, 센서 종류/통신타입 enum 제공.

#### [SensorFT.cpp](CRobot/SensorFT.cpp) / [.h](CRobot/SensorFT.h) (358줄) — **힘/토크 센서 `CSensorFT`**

6축 F/T(Fx,Fy,Fz,Tx,Ty,Tz) 처리. **`ProcessRawData()` 파이프라인**: raw INT16 → 나누기(힘÷50, 토크÷2000)로 물리값화 → 바이어스 제거 + 스케일 적용(`ApplyCalibration`) → 크기 계산 → 과부하 검사. `ZeroBias()`는 100샘플 평균으로 바이어스 산출. 캘리브레이션 바이너리 파일 저장/로드.

#### [SensorNRMKEndTool.cpp](CRobot/SensorNRMKEndTool.cpp) / [.h](CRobot/SensorNRMKEndTool.h) (364줄) — **NRMK 엔드툴 `CSensorNRMKEndTool`**

엔드이펙터(EOAT, EtherCAT 슬레이브 #6) 통합. F/T 데이터를 슬레이브에서 읽고(`ReadData`), **LED 제어**(`LED_RED/GREEN/BLUE/YELLOW/OFF`, 깜빡임), **그리퍼**(`SetGripper/GetGripper`), 버튼 상태, F/T 에러/과부하 플래그. `UpdateLEDStatus()`가 센서 상태에 따라 LED 자동 갱신(에러→빨강, 과부하→노랑, 정상→녹색) — `Indy7Ctrl`이 제어모드 표시에도 활용.

#### [ConfigRobot.cpp](CRobot/ConfigRobot.cpp) / [.h](CRobot/ConfigRobot.h) (482줄) — **설정 로더 `CConfigRobot`**

`.cfg` 파일을 파싱해 구조체로 제공. `ReadSystemConfig`(이름/시뮬/텔레옵/URDF경로/시작시제어모드/관절수/KP·KD), `ReadRTTaskConfig`(6개 태스크), `ReadEtherCATConfig`(마스터 + 슬레이브별 벤더ID·기어비·토크상수·엔코더·홈잉·각종 리미트). `GetSystemConf/GetTaskList/GetEcatSlaveList/...`로 접근. `WriteLastPosition()`으로 종료 시 위치 저장. (실제 INI 파싱은 Global의 ConfigParser 사용.)

#### [ExtInterface.cpp](CRobot/ExtInterface.cpp) / [.h](CRobot/ExtInterface.h) (658줄) — **외부 통신 `CExtInterface`**

**TCP 서버(포트 7420)**로 외부 클라이언트(예: GUI/티치펜던트)와 바이너리 프로토콜 통신.

- **패킷 구조**: `STX(0x02 0x5B)` + SpinID + 길이 + Cmd + SubCmd + Data + **CRC16(ARC)** + `ETX(0x5D 0x03)`
- **수신**: `ParseRecvPacket`(STX 탐색·길이·ETX·CRC 검증) → `InterpretPacket` → Cmd별 분기(`CMD_ROBOT/AXIS/ECAT_MASTER/ECAT_SLAVE`). 축 위치 설정(`SUBCMD_SET_POS`) 명령은 콜백으로 `CRobot::OnRecvAxisCommand`에 전달.
- **송신**: 클라이언트 접속 시 로봇/마스터/축 메타데이터 전송, `SendAck/Nack`, EtherCAT 상태 브로드캐스트.
- 소켓 콜백은 `std::bind`로 등록, CRC 무결성 검증.

---

### 4-D. 공용 유틸 (`Global/`)

#### [ConfigParser.cpp](Global/ConfigParser.cpp) / [.h](Global/ConfigParser.h) (242줄)

Windows `GetPrivateProfileString/Int` 호환의 **INI 파서**. `[섹션] 키=값` 읽기/쓰기, 파일/디렉토리 존재 확인(`IsExistFile`), 쓰기 시 임시파일+rename으로 안전 갱신. `CConfigRobot`이 이걸로 `.cfg`를 읽음.

#### [CRC16.cpp](Global/Comm/CRC16.cpp) / [.h](Global/Comm/CRC16.h) (128줄)

설정 가능한 **CRC-16 체크섬**(다항식/초기값/리플렉트/XOR). 룩업테이블 기반 고속 계산. 파생 클래스 `CRC16_ARC`(Modbus/ARC 표준: 0x8005). `ExtInterface`의 패킷 무결성 검증에 사용.

#### [Socket.cpp](Global/Comm/Socket.cpp) / [.h](Global/Comm/Socket.h) (613줄)

Linux 소켓 래퍼. `CSocket`(생성/바인드/리슨/액셉트), `CTcpServer`(다중 클라이언트, 수락 스레드 + 죽은 클라이언트 정리 스레드, `SendToClients` 브로드캐스트), `CTcpClient`(송신 큐 기반 비동기, 수신/송신 스레드 분리). `std::function` 콜백 이벤트 구조. `ExtInterface`의 통신 기반.

#### [Defines.h](Global/Defines.h) (전역 정의)

모든 파일이 포함하는 헤더. **로깅 매크로**(`DBG_LOG_INFO/ERROR/WARN`, 컬러 출력), **타입**(`BOOL/BYTE/TSTRING/UINT32`…), **통신 구조체**(`ST_AXIS_CMD/STATE`, `ST_ECAT_METADATA` — 1바이트 패킹), **시간 함수**(`GetCurrTimeInNs` CLOCK_MONOTONIC), **각도/엔디언 변환**(`ConvertDeg2Rad`, `ConvertS32ToByteArrayBE`…, 빅엔디언), 키입력(`getch`). `main.cpp`의 `cpu_relax()`, `RET_SUCC/FAIL`도 여기(또는 posix_rt).

---

## 5. 실행 시퀀스 요약

1. `./Indy7Ctrl.out -f INDY7.cfg` → main이 cfg 읽고 `CRobotIndy7` 생성(생성자에서 6개 RT 태스크 함수 등록)
2. `Init()` → 동역학 제어기 생성·URDF 로드 → 비전 Init → 외부I/F(7420) 시작 → **EtherCAT 마스터/7슬레이브 초기화** → RT 태스크 6개 기동
3. 1ms마다 `proc_ethercat_control`이 슬레이브 I/O, `proc_main_control`이 **q 읽기 → 토크 계산 → MoveTorque** 반복
4. 키보드로 제어모드 전환(중력보상↔CTC↔IK↔비전서보), 로깅/테스트 트리거
5. Ctrl+C → `DeInit()`: 태스크 정지, 종료위치 저장, EtherCAT 해제

---

## 6. 코드를 처음 보는 분께 — 실무 주의점

- **경로 하드코딩**: 소스 곳곳과 cfg에 `/home/raimlab/RAON-RT/...`가 박혀 있음(URDF, 로그 폴더 `rt_log_results`/`iso_csv`/`rt_ik_error_log`, ViSP 빌드 경로). 본인 경로로 바꿔야 정상 동작·로깅됨.
- **기본 설정파일명이 `ELMO.cfg`** ([ConfigRobot.h:14](CRobot/ConfigRobot.h#L14)) — 반드시 `-f INDY7.cfg`로 실행.
- **적응제어(eAdaptiveControl)는 미구현** — 선택해도 중력보상으로 동작.
- 코드 주석에 `// jieun`, `// trash code! spaghetti code!!` 등 작업 흔적이 남아 있음(IK/검증 관련 실험 코드 영역).
- 외부 의존성(NRMK EMasterApp, rtposix, RBDL, ViSP, RealSense)이 선설치돼 있어야 빌드 가능 — 저장소만으로는 빌드되지 않음.
