# Kinect V2のキャリブレーションファイル保存場所
![img](.images/header.jpg)

キャリブレーションをしたらここに保存しておくこと。

## clone 

```bash
cd catkin_ws/src/iai_kinect2/kinect2_bridge/
rm -r data #本家が残しているファイルと置き換える。削除する際は気をつける
git clone git@github.com:Nishida-Lab/kinect2_calibration.git data
```

## Kinect V2のキャリブレーションの仕方
iai_kinectの公式リポジトリのREADMEを見る

> [code-iai/iai_kinect2](https://github.com/code-iai/iai_kinect2)  
> Kinect2 Calibration  
> [https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration)

それぞれframeは10回とってやってます。  
キャリブレーションファイルの保存場所は`catkin_ws/src/iai_kinect2/kinect2_bridge/data/508594442542`という感じです。
