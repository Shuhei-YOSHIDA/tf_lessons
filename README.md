tf_lessons
====

ROSのTFの利用に関しての個人的な知見について記したレポジトリ

## tf overview
tfはROSを用いたシステムにおいて，分散した各ノードで
座標系ツリーと時間に対する補間を管理するためのライブラリである．
tfとtf2があるが互換性があるので新規にtfを用いたコードを扱う場合はtf2を用いればよい
ただ，古い方のtfでは座標系管理部分とROSとのインターフェースの部分が混在してたが，
tf2ではそれらが分割されてそれぞれROSパッケージ "tf2", "tf2_ros" となっている．
大抵の場合は"tf2_ros"パッケージを呼べば十分であり，"tf2"パッケージを用いる場合は
C++でtf2::Quaternionのような型や座標変換のユーティリティコードを単独で呼ぶくらいだと思われる．
ただ，そのような座標変換はEigenを使うほうがネット上で情報を多く見つけやすく，
tf2での型とEigenの型を変換するtf2_eigenのなどのパッケージがあるのでEigenを用いたほうが良いだろう．

tfを扱うにあたっては，
* (基本的に)各座標系が一つのツリーの要素になっている
* 座標系ツリーの中にループする部分がない
* 座標系の変化には管理される期間がある

ということに留意する必要がある．

## static tf
### Broadcast and Listen
* Static TFは時間的に不変として扱われるTF

## non-static tf
### Broadcast and Listen

## static_transform_publisher node

## URDF + joint_state_publisher + robot_state_publisher

## Keep one tf-tree
### Representative example of frames
/world, /odom, /base_link

### Connect fragments in ROSBAG data



