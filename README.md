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

tfによる座標変換と補間は，ある時刻のある座標系間の座標変換をbroadcast/listenすることで行われる．
broadcast/listenは実態はpublish/subscribeであり，座標変換情報をtopic経由で出力/入力することを指す．
listenerにはデフォルトで過去10秒分の座標系変換情報が保持されており，
所望の時刻のある座標系間の変換を，そのデータを用いて時間補間により計算することができる．
そのため，古すぎる時刻，座標系ツリーにデータ中に途切れのある変換の指定，
そして補間のためには時刻的に2点で挟む必要があるので最新データでも過去になる
今現在の時刻の変換が得ることができないという特徴がある．
(座標変換は保存されたデータの内挿で計算され外挿では行われない)

ただ，実用上は常に座標系ツリーの中に既に固定された座標系変換というものを考えたほうが都合がよい．
そのために/tf_staticというtopicが用意されており，固定された変換はここにbroadcastされるのが基本となっている．
これは周期的にtopic /tf_staticにpublish(broadcast)されているわけではなく，
このtopicに対するsubscribe(listener)が現れるたびにpublishするという実装になっている．
詳しくはtf2_ros packageのソースstatic_broadcast_publisher.cppを見てみると良いだろう．
たいてい無駄ではあるが，固定された座標変換を周期的に/tfにbroadcastし続けても結果としては殆ど同じである．

## static tf
### Broadcast and Listen
* Static TFは時間的に不変として扱われるTFであり/tf_static

## non-static tf
### Broadcast and Listen

## static_transform_publisher node

## URDF + joint_state_publisher + robot_state_publisher

## Keep one tf-tree
### Representative example of frames
/world, /odom, /base_link

### Connect fragments in ROSBAG data



