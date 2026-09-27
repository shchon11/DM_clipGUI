# 온라인 보정 탭·3D 뷰어 검증 기록

이 문서는 라이브 solver 훅 연결 전의 UI·합성 데모 검증 기록이다.
실제 계산 스트림 연결 이후의 실행·동일성·성능 증거는 [라이브 검증 기록](calib_live_validation.md)을 따른다.

검증일: 2026-09-26. 입력 `/hdd/DM_calib/nt_regress/full/{work,out}`는 읽기 전용으로 사용했다.
전체 clip GUI, 센서, 레코더, 실제 보정 solver는 실행하지 않았다.

## 자동 검사

```bash
QT_QPA_PLATFORM=offscreen OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 -m pytest -q test tests
python3 -m pyflakes scripts/calib_viz scripts/calib_viz_demo.py scripts/calib_tab_demo.py \
  scripts/calib_tab.py scripts/online_calib.py test/test_calib_viz_*.py \
  test/test_calib_tab_gui.py tests/test_calib_viz_*.py
python3 -m compileall -q scripts/calib_viz scripts/calib_tab.py scripts/online_calib.py \
  scripts/clip_gui.py scripts/calib_viz_demo.py scripts/calib_tab_demo.py
git diff --check
```

**100 passed** (10.91초). 변경 전 기존 테스트는 52 passed였다.
Pyflakes, Python 문법 검사, diff 공백 검사 통과. 별도 정적 타입 검사 설정은 없다.
`clip_gui.py` 전체 Pyflakes에는 기존 미사용 import `QPixmap`, `QScrollArea` 2건이 있으며,
이번 변경은 종료 시 `self.calib.shutdown()` 한 줄이다. ROS/C++ 전체 빌드는 수행하지 않았다.

주요 회귀 범위:

- 실제 설계/warm-start와 최종 YAML 그대로 표시, 중간 지표 미생성, 즉시 최종 전환.
- 설치/사용자 실행 파일의 설정 출처, 설정 불명확 시 미표시, 시작 시 초기값 캐시 갱신.
- 실제 진행·추출/LO 구간 수·ETA 유지, 카메라×구간 작업 수와 구간 수 구분.
- 늦게 생긴 스트림 연결, 잘못된 manifest 복구, 실작업의 합성 스트림 거절.
- 완료 결과가 오래된 중간 포즈보다 우선함, 작업 변경 시 이전 화면·이력 초기화.
- 실제 야간 결과 16대 gate 통과, RGB 참고 검사 14개, 열화상 실패·미판정·전체 배치 실패 구분.
- 선택한 영상만 읽기, 새 이벤트 없이 카메라 변경, 누락 자산 복구, worker/timer 종료.
- 전체 구간 순회, 독립 좌표계 유지, 제한된 점·특징·자산 캐시, 누락 영상 미표시.

## 오프스크린 화면

Xvfb/GLX + llvmpipe, `LP_NUM_THREADS=2`, OMP/BLAS 각 1, 화면 갱신 최대20FPS.

| 화면 | 파일 | 구간 / 표시점 |
|---|---|---|
| 실제 완료 결과를 연 온라인 보정 탭 | [calib_tab_embedded.png](img/calib_tab_embedded.png) | 포즈·지표16대 / viz 미발행이므로 지도·영상 대기 |
| 합성 리플레이 초기 | [calib_viz_01_006.png](img/calib_viz_01_006.png) | S02 / 16,190 |
| 합성 리플레이 중간 | [calib_viz_02_055.png](img/calib_viz_02_055.png) | W03 / 23,597 |
| 리플레이 실측 최종값 | [calib_viz_03_100.png](img/calib_viz_03_100.png) | W14 / 27,580 |
| 열화상 우 선택 | [calib_viz_thermal.png](img/calib_viz_thermal.png) | S01 / 28,401 |

[탭 증거](img/calib_tab_embedded.json): 실제 최종 결과, 16대 통과, 참고 배지14개.
[리플레이 측정](img/calib_viz_capture.json): 16대 선택 가능, 약19.2–20.0FPS.
[열화상 측정](img/calib_viz_thermal_capture.json),
[실제 Qt 조작 검사](img/calib_viz_interaction.json),
[시각 검토](img/calib_viz_visual_verdict.json).
실제 조작 검사는 카메라16대 각각의 투영, 메뉴/센서 동기화, 일시정지/재개,
숨김 시 I/O 중지, 종료 후 worker/timer 해제를 확인했다.

3개 리플레이 캡처의 `/usr/bin/time -v`: 22.75초, 최대 RSS354,688KiB(346.4MiB),
평균 CPU81%, swap0. 소프트웨어 렌더링을 포함한 짧은 캡처이며 장시간 실차 성능은 미검증이다.
GL expose/창 크기 변경도 `frameSwapped`를 발생시키므로 표시 FPS는 설정된 timer cap보다
일시적으로 높을 수 있다.

재생 producer만 실제25개 구간을 순회한 별도 측정:
27스냅샷, 모든 구간/모든16대 영상, 준비0.088초, 전체110.18초,
최대 RSS100.3MiB, 잔류 자산3세트/9.2MiB. 초기 디스크 읽기가 포함된 측정이다.
결과는 도구 출력에서 기록했으며 임시 데이터는 종료 시 삭제했다.
[측정 기록](img/calib_viz_replay_budget.json).

## 재현

```bash
# 실작업 결과를 담은 탭만 실행; 임시 JobStore 사용, 센서/레코더 없음
xvfb-run -a -s '-screen 0 1920x1400x24 +extension GLX' \
  env QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  python3 scripts/calib_tab_demo.py --workdir /hdd/DM_calib/nt_regress/full/work \
  --screenshot docs/img/calib_tab_embedded.png

# 리플레이 캡처
xvfb-run -a -s '-screen 0 1920x1200x24 +extension GLX' \
  env QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  python3 scripts/calib_viz_demo.py --replay /hdd/DM_calib/nt_regress/full/work \
  --screenshots docs/img --max-fps 20 --max-points 30000
```

Qt의 `offscreen` 플랫폼은 GL 컨텍스트를 제공하지 않아 이미지 캡처는 Xvfb로 수행했다.
실제 solver 동시 실행 시 처리량 변화는 측정하지 않았다. 실제 작업은 원본 점군을 처리하지 않고
작은 이벤트/출력만 읽으며, 지도·영상은 solver가 viz 스트림을 발행할 때에만 나타난다.
기존 소스의 중간 포즈/불확실도를 복원했다고 주장하지 않는다.
