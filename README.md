# ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**13 Jan 2017**: OpenCV 3와 Eigen 3.3 지원

**22 Dec 2016**: AR demo 추가 (see section 7).

ORB-SLAM2은 **Monocular**, **Stereo** 와 **RGB-D** 카메라용 실시간 SLAM library이다. camera trajectory와 3D reconstruction을 계산한다.(stereo와 RGB-D의 경우 실제 scale을 사용한다.) 실시간으로 loops detection이 가능하고 camera relocalize가 가능하다. stereo나 monocolur로 [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) 내에 SLAM 시스템을 실행하는 예제를 제공한다. RGB-D나 monocular에서 [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular 예제 그리고 stereo나 monocular에서 [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)를 사용한 예제를 제공한다. live monocular, stereo, RGB-D streams을 처리하는 ROS node를 제공한다. **ROS 없이도 library를 컴파일할 수 있다** ORB-SLAM2은 *SLAM Mode* 와 *Localization Mode* 사이에서 변경할 수 있는 GUI를 제공한다. (section 9)

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### 관련 논문:

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM2는 [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt)를 따른다. 모든 코드와 의존 라이브러리에 대한 목록 및 라이센스는 [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md) 참조한다.

상업적 사용 목적의 ORB-SLAM2의 소스에 대한 문의는 저작자에게 : orbslam (at) unizar (dot) es.

ORB-SLAM2 (Monocular) 교육목적으로 사용하는 경우 아래 인용:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

ORB-SLAM2 (Stereo or RGB-D) 교육목적으로 사용하는 경우 아래 인용:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

# 2. 사전 요구사항
테스트된 환경 : **Ubuntu 12.04**, **14.04**, **16.04**
컴퓨터 성능이 좋은 경우 실시간 성능과 안정적인 정확도를 제공한다.

## C++11 or C++0x Compiler
C++11의 새로운 thread와 chrono 기능을 사용한다.

## Pangolin
시각화와 UI를 위해서 [Pangolin](https://github.com/stevenlovegrove/Pangolin) 를 사용한다. 다운로드 및 설치는 https://github.com/stevenlovegrove/Pangolin 참고

## OpenCV
이미지 처리 관련 [OpenCV](http://opencv.org)를 사용한다. **최소 : 2.4.3 이상. 테스트된 환경 : OpenCV 2.4.11 and OpenCV 3.2**

## Eigen3
g2o(아래 참고)에서 사용. 다운로드 및 설치는 http://eigen.tuxfamily.org 참고. **최소 3.1.0 이상**

## DBoW2 and g2o (Included in Thirdparty folder)
[DBoW2](https://github.com/dorian3d/DBoW2) library를 수정하여 place recognition을 수행하고 [g2o](https://github.com/RainerKuemmerle/g2o) library로 non-linear optimization을 수행한다. 이 2개 수정된 라이브러리는 *Thirdparty* 폴더에 포함되어 있다.

## ROS (optional)
[ROS](ros.org)를 사용해서 monocular, stereo, RGB-D 카메라의 live input을 처리하는 예제를 제공한다. 이 예제를 빌드하는 것은 옵션이다. ROS를 사용하려면 Hydro 이상의 버전이 필요하다.
# 3. ORB-SLAM2 library와 예제 빌드하기

저장소 clone :
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

`build.sh` 스크립트를 제공하여 *Thirdparty* libraries 와 *ORB-SLAM2*를 빌드한다. 필요한 모든 의존성을 설치했는지 확인한다. (section 2). 실행 :

```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

이렇게 하면 **libORB_SLAM2.so** 가 *lib* 폴더에 생성되고 **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** 가 *Examples* 폴더에 생성된다.

# 4. Monocular 예제

## TUM Dataset

1. http://vision.in.tum.de/data/datasets/rgbd-dataset/download에서 다운받고 압출 풀기


2. 아래 명령을 실행. `TUMX.yaml`를 TUM1.yaml,TUM2.yaml or TUM3.yaml로 변경하여 freiburg1, freiburg2 and freiburg3 sequences에 대해 실행한다. `PATH_TO_SEQUENCE_FOLDER`를 압축 풀린 시퀀스 폴더로 변경한다.

```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. http://www.cvlibs.net/datasets/kitti/eval_odometry.php의 dataset((grayscale images)을 다운받는다.

2. 아래 명령을 실행한다. `KITTIX.yaml`을 KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml로 변경하여 sequence 0 to 2, 3, and 4 to 12에 대해 실행한다. `PATH_TO_DATASET_FOLDER`를 압축 풀린 시퀀스 폴더로 변경한다. `SEQUENCE_NUMBER`를 00, 01, 02,.., 11로 변경한다.

```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets에서 sequence를 다운받기(ASL format)

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
아래 첫번째 명령을 V1 and V2 sequences에 대해 실행하거나 두번째 명령은 MH sequences에 대해 실행한다. PATH_TO_SEQUENCE_FOLDER와 SEQUENCE를 실행하고자 하는 시퀀스에 맞게 변경한다.

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

# 5. Stereo 예제

## KITTI Dataset

1. http://www.cvlibs.net/download.php?file=data_odometry_gray.zip에서 dataset (grayscale images) 다운받기

2. 아래 명령 실행. `KITTIX.yaml`을 KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml로 변경하여 sequence 0 to 2, 3, and 4 to 12에 대해 실행한다. `PATH_TO_DATASET_FOLDER`를 압축 풀린 시퀀스 폴더로 변경한다. `SEQUENCE_NUMBER`를 00, 01, 02,.., 11로 변경한다.

```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets 에서 sequence를 다운받기(ASL format)

2. V1과 V2 sequences에 대해서 아래 첫번째 명령을 실행하거나 MH sequences에 대해서 2번째 명령실행. PATH_TO_SEQUENCE_FOLDER와 SEQUENCE를 실행하고자 하는 시퀀스에 맞게 변경한다.

```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

# 6. RGB-D 예제

## TUM Dataset

1. http://vision.in.tum.de/data/datasets/rgbd-dataset/download에서 다운받고 압출 풀기

2. RGB images와 depth images를 python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools)를 사용해서 연결한다. 이미 *Examples/RGB-D/associations/* 폴더에 몇개의 시퀀스에 대한 연결을 제공한다. 아래 명령을 실행해서 자신과 관련된 파일을 생성한다.

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. 아래 명령을 실행한다. `TUMX.yaml`을 TUM1.yaml,TUM2.yaml로 변경하거나 TUM3.yaml을 freiburg1, freiburg2 and freiburg3 sequences에 대해서 변경한다. `PATH_TO_SEQUENCE_FOLDER`를 압축을 푼 sequence 폴더로 변경한다. `ASSOCIATIONS_FILE`을 관련된 파일에 대한 path로 변경한다.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 7. ROS 예제

### mono, monoAR, stereo and RGB-D를 위한 nodes 빌드하기

1. ROS_PACKAGE_PATH 환경 변수에 *Examples/ROS/ORB_SLAM2*를 포함한 path를 추가한다. .bashrc 파일을 열고 아래 줄을 추가한다. PATH는 ORB_SLAM2를 clone한 폴더로 변경한다.

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```
  
2. `build_ros.sh` script 실행:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Monocular Node 실행
topic `/camera/image_raw`으로부터 monocular input을 위해서 ORB_SLAM2/Mono node를 실행한다. vocabulary file과 settings file을 제공해야 한다. monocular 예제를 참고한다.

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Monocular Augmented Reality 데모
이 데모는 augmented reality의 예제이다. interface를 사용해서 scene의 planar region에 가상의 cubes를 삽입할 수 있다. node는 topic `/camera/image_raw`로부터 image를 읽는다.

  ```
  rosrun ORB_SLAM2 MonoAR PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Stereo Node 실행하기
topic `/camera/left/image_raw` and `/camera/right/image_raw`로부터 stereo input을 위해서 ORB_SLAM2/Stereo node를 실행한다. vocabulary file과 settings file을 제공해야 한다. 만약 **rectification matrices를 제공**한다면 (Examples/Stereo/EuRoC.yaml 예제 참고), node는 online으로 images를 rectify해야한다. **만약 rectification matrices를 제공하지 않는다면 images는 pre-rectified 되어야 한다.**

  ```
  rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
**Example**: EuRoC dataset(http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)에서 rosbag을 다운받는다. 터미널에 3개 탭을 열고 각 탭에서 아래 명령ㅇ르 실행한다.:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```

일단 ORB-SLAM2이 vocabulary를 로드하면 rosbag tab에서 space를 누른다. 즐긴다! Note: 이 데이터셋을 실행하기 위해서는 강력한 컴퓨터가 필요하다.  

### RGB_D Node 실행하기
topic `/camera/rgb/image_raw`와 `/camera/depth_registered/image_raw`로부터 RGB-D input을 위해서 ORB_SLAM2/RGBD node를 실행한다. vocabulary file과 settings file을 제공해야 한다. 위에 RGB-D 예제를 참고한다.

  ```
  rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
# 8. 자신만의 sequences 처리하기
여러분의 camera의 calibration에 대한 setting 파일을 생성해야한다. monocular, stereo and RGB-D camera에 대한 TUM과 KITTI datasets에서 제공한 setting file을 참고한다. 우리는 OpenCV의 calibration model을 사용한다. 예제에서 ORB-SLAM2 library를 사용하는 프로그램을 생성하고 SLAM 시스템에 image를 전달하는 방법을 배운다. stereo input은 동기화되고 rectified되어야 한다. RGB-D input은 동기화되고 depth registered되어야 한다.

# 9. SLAM과 Localization Modes
map viewer의 GUI를 이용해서 *SLAM* 과 *Localization mode* 사이에 스위칭할 수 있다.

### SLAM Mode
이것이 기본 모드이다. 시스템은 3개 thread로 병렬로 실행된다.: Tracking, Local Mapping과 Loop Closing. 시스템은 카메라를 localize하고 새로운 map을 생성하고 close loops를 시도한다.

### Localization Mode
작업 영역에서 좋은 품질의 map을 가지고 있는 경우에 사용할 수 있다. 이 모드에서 Local Mapping과 Loop Closing은 비활성화된다. 시스템은 map(더 이상 업데이트가 없는)내에서 카메라를 localize하며 필요에 따라ㅣ서는 relocalization을 사용한다.
