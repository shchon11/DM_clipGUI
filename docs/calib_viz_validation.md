# 3D 보정 뷰어 검증 기록

검증일: 2026-09-26. Python3.10 / PyQt5 / PyQtGraph0.13.7 / PyOpenGL3.1.7.
실제 입력: `/hdd/DM_calib/nt_regress/full/{work,out}` (읽기 전용).

- `python3 -m pytest -q tests/test_calib_viz_core.py tests/test_calib_viz_viewer.py`: **26 passed**.
- `python3 -m pyflakes scripts/calib_viz scripts/calib_viz_demo.py tests/test_calib_viz_core.py tests/test_calib_viz_viewer.py`: 통과.
- `python3 -m compileall -q scripts/calib_viz scripts/calib_viz_demo.py`, `git diff --check`: 통과.
- 별도 정적 타입 검사 설정은 없는 Python 모듈이다. 타입 검사 도구를 실행했다고 주장하지 않는다.

실측 최종16대의 앞/좌/상 축 부호, rig.yaml 위치 일치, KB4/OpenCV 투영 동치,
열화상 plumb_bob 투영, 최종 지표·판정 원본 일치를 검증했다.
부분 JSON, 중복 seq, 파일 교체/재기록, 큰 로그의 최신 부분 읽기, 잘못된 필드·NPZ·경로,
누락 영상의 마지막 정상 프레임 유지, 재시작 시 지도 교체, 종료 시 timer/thread 해제를 검사했다.

Xvfb/GLX 1600×1000 렌더링:

| 화면 | 파일 | 표시 점 개수 | 측정 FPS |
|---|---|---:|---:|
| 초기 추정 | [calib_viz_01_006.png](img/calib_viz_01_006.png) | 4,755 | 38.3 |
| RGB 수렴 중 | [calib_viz_02_055.png](img/calib_viz_02_055.png) | 56,801 | 40.0 |
| 최종 검증 | [calib_viz_03_100.png](img/calib_viz_03_100.png) | 56,801 | 40.0 |
| 열화상 선택 | [calib_viz_thermal.png](img/calib_viz_thermal.png) | 56,801 | 별도 캡처 |

[측정 JSON](img/calib_viz_capture.json)은 실제 GL frameSwapped 기준이다.
`LIBGL_ALWAYS_SOFTWARE=1`의 렌더러는 **llvmpipe (LLVM15.0.7,256bits)**였다.
3화면 캡처의 `/usr/bin/time -v` 측정: 17.83초, 최대 RSS338,648KiB(약331MiB),
평균 CPU122%(약1.22코어), swap0. 이는 짧은 대표 캡처이며 장시간/실차 성능 시험은 아니다.

[조작 검증 JSON](img/calib_viz_interaction.json): 측면 영상 선택 시1,578개,
열화상 선택 시574개 투영점을 확인했다. 영상 메뉴·센서 선택 동기화, 초기 추정 비교,
일시정지/재개·종료를 실제 Qt/OpenGL 창에서 확인했다.
[시각 검토 기록](img/calib_viz_visual_verdict.json)은 외부 디자인 참조 없이 요구사항과 이전 렌더를 기준으로 한다.

현재 NVIDIA 드라이버 접근 불가로 RTX3080Ti/4060Ti 성능은 미검증이다.
solver 발행 훅이 아직 없으므로 실제 온라인 solver와의 종단 간 연결도 미검증이다.
지도는 S01 대표 샘플, 영상은3대, 중간 포즈/불확실도는 합성이다. 열화상 rolling shutter 생략 등
정확한 범위와 발행 훅 계약은 [스트림 규약](calib_viz_stream.md)을 따른다.
