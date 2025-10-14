# AUTOCARZI

Unity 기반 청각장애인을 위한 자율주행 인터페이스 연구 프로젝트

## Git LFS 설정

이 프로젝트는 용량이 큰 에셋 파일들을 포함하고 있어 Git LFS를 사용합니다.

### 설치 방법

```bash
# macOS
brew install git-lfs

# Windows
# Git for Windows에 포함되어 있거나 https://git-lfs.github.io/ 에서 다운로드

# Ubuntu/Debian
sudo apt install git-lfs
```

### 초기화

```bash
git lfs install
git clone https://github.com/AUTOCARZI/AUTOCARZI.git
cd AUTOCARZI
```

## LaneDetection 환경 설정

### 필수 패키지 설치

```bash
pip install -r requirements.txt
```

requirements.txt 파일 위치: `Assets/Azerilo/Car Model No.1201 Asset/Scripts/AI/requirements.txt`

### 서버 실행

```bash
python "Assets/Azerilo/Car Model No.1201 Asset/Scripts/AI/LaneDetectionServer.py"
```

터미널을 종료하지 말고 서버를 실행한 상태로 유지해야 합니다.

## 프로젝트 실행

### 시작 Scene

`Assets/Scenes/A/RoadScene-Scenario-A.unity`

### 조작 방법

- **Tab 키**: 다음 Scene으로 이동 (24개의 Scene이 임의의 순서로 배치됨)
- **Enter 키**: Scene 일시정지 및 반응 시간 기록 (인터페이스 발동 시점부터 Enter 키 입력 시점까지의 타이머 측정)

## 데이터 기록

반응 시간 기록 파일 저장 경로: `Assets/ReactionTime_Records/`
