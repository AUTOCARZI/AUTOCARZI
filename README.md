# Unity 프로젝트 Git LFS 가이드

## 📋 개요
이 프로젝트는 **Git LFS (Large File Storage)**를 사용하여 큰 바이너리 파일들(.dylib, .dll, 텍스처, 모델 등)을 관리합니다.

## 🚀 처음 프로젝트 클론하기

### 1. Git LFS 설치 (필수!)
```bash
# macOS (Homebrew)
brew install git-lfs

# Windows (Git for Windows에 포함됨)
# 또는 https://git-lfs.github.io/ 에서 다운로드

# Ubuntu/Debian
sudo apt install git-lfs
```

### 2. Git LFS 초기화
```bash
git lfs install
```
> ⚠️ **주의**: 이 명령은 **한 번만** 실행하면 됩니다. (컴퓨터당 1회)

### 3. 프로젝트 클론
```bash
git clone https://github.com/AUTOCARZI/AUTOCARZI.git
cd AUTOCARZI
```

### 4. LFS 파일 다운로드 확인
```bash
# LFS 파일들이 제대로 다운로드되었는지 확인
git lfs ls-files

# LFS 상태 확인
git lfs status
```

---

## 📁 현재 LFS로 관리되는 파일 유형

다음 파일들이 자동으로 LFS로 관리됩니다:

### Unity 바이너리 파일
- `*.dylib` (macOS 동적 라이브러리)
- `*.dll` (Windows 동적 라이브러리)
- `*.so` (Linux 동적 라이브러리)
- `*.bundle` (Unity 번들 파일)

### 3D 모델 및 텍스처 (설정된 경우)
- `*.fbx`, `*.obj`, `*.blend`
- `*.png`, `*.jpg`, `*.tga`, `*.psd`
- `*.mp3`, `*.wav`, `*.ogg`
- `*.mp4`, `*.mov`, `*.avi`

---

## 💻 개발 워크플로우

### 새로운 작업 시작 전
```bash
# 1. 최신 변경사항 받기
git pull

# 2. LFS 파일들도 최신으로 업데이트
git lfs pull
```

### 커밋하기 전
```bash
# 1. 상태 확인
git status

# 2. LFS 상태 확인 (큰 파일이 추가되었다면)
git lfs status

# 3. 파일 추가
git add .

# 4. 커밋
git commit -m "feat: 새로운 기능 추가"

# 5. 푸시
git push
```

### 큰 파일을 새로 추가할 때
```bash
# 새로운 파일 유형을 LFS로 추적하고 싶다면
git lfs track "*.새로운확장자"

# .gitattributes 파일 커밋
git add .gitattributes
git commit -m "chore: add new file type to LFS tracking"
```

---

## 🔧 문제 해결

### "Git LFS not found" 에러가 날 때
```bash
# Git LFS 재설치
git lfs install --force
```

### LFS 파일이 다운로드되지 않을 때
```bash
# 강제로 LFS 파일들 다운로드
git lfs fetch --all
git lfs checkout
```

### 저장소 크기가 너무 클 때
```bash
# LFS 파일들만 확인하고 싶다면
git lfs ls-files

# 전체 저장소 크기 확인
git count-objects -vH
```

### Unity에서 Library 폴더 문제
Library 폴더는 Git에서 무시됩니다 (.gitignore에 포함됨).
프로젝트를 열 때 Unity가 자동으로 재생성합니다.

---

## 📝 주의사항

### ✅ 해야 할 것
- 프로젝트 클론 전에 **반드시 Git LFS 설치**
- 큰 바이너리 파일 추가 시 LFS 추적 설정
- 정기적으로 `git lfs pull` 실행

### ❌ 하지 말 것
- LFS 설치 없이 클론하기
- Library 폴더를 Git에 추가하기
- 텍스트 파일(.cs, .js, .json 등)을 LFS로 추적하기

### 💡 팁
- Unity 프로젝트는 첫 열기 시 시간이 걸릴 수 있습니다 (Library 폴더 생성)
- 큰 파일 작업 시 네트워크 상태 확인
- 브랜치 전환 시 `git lfs checkout` 실행 권장

---

## 🆘 도움이 필요할 때

### 유용한 명령어들
```bash
# LFS 설정 확인
git lfs env

# LFS로 관리되는 파일 목록
git lfs ls-files

# LFS 사용량 확인
git lfs status

# 특정 파일이 LFS인지 확인
git lfs pointer --file="파일명"
```

### 참고 링크
- [Git LFS 공식 문서](https://git-lfs.github.io/)
- [Unity Git 가이드](https://docs.unity3d.com/Manual/VersionControl.html)
- [Git LFS 튜토리얼](https://github.com/git-lfs/git-lfs/wiki/Tutorial)

---

**문서 업데이트:** 2025-06-05