tf_lessons
====

ROSのTFの利用に関しての個人的な知見について記したレポジトリ

## tf overview
tfはROSを用いたシステムにおいて，分散した各ノードで
座標系ツリーと時間に対する補間を管理するためのライブラリである．
tfとtf2があるが互換性があるので新規にtfを用いたコードを扱う場合はtf2を用いればよい．
ただ，古い方のtfでは座標系管理部分とROSとのインターフェースの部分が混在してたが，
tf2ではそれらが分割されてそれぞれROSパッケージ "tf2", "tf2_ros" となっている．
大抵の場合は"tf2_ros"パッケージを呼べば十分であり，"tf2"パッケージを用いる場合は
C++でtf2::Quaternionのような型や座標変換のユーティリティコードを単独で呼ぶくらいだと思われる．
ただ，そのような座標変換はEigenを使うほうがネット上で情報を多く見つけやすく，
tf2での型とEigenの型を変換するtf2_eigenのなどのパッケージがあるのでEigenを用いたほうが良いだろう．

tfを扱うにあたっては，
* 座標系の変化には管理される期間がある
* (基本的に)各座標系が一つのツリーの要素になっている
* 座標系ツリーの中にループする部分がない

ということに留意する必要がある．

tfによる座標変換と補間は，ある時刻のある座標系間の座標変換をbroadcast/listenすることで行われる．
broadcast/listenは実態はpublish/subscribeであり，座標変換情報をtopic経由で出力/入力することを指す．
listenerにはデフォルトで過去10秒分の座標系変換情報がバッファに保持されており，
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

以下では，実際にtfを実行しながら解説する．
```
$ # cloneして自分のROS workspaceに本パッケージをビルドすること
$ roscore
$ rviz -d tf_lessons/rviz/lesson.rviz #別端末で実行
```
すること．

## Broadcast and Listen for static TF
`lesson1.cpp`は簡単なtfをbroadcastする例である．
"world"座標系から"base_link"座標系への変換をbroadcastしている．

`lesson1.cpp`はlesson1というnode名であり，引数を付けて次の通り実行する．
```
$ rosrun tf_lessons lesson1 static
```
このとき，`tf2_ros::StaticTransformBroadcaster`を用いて，/tf_staticに座標変換をbroadcastしている．
RViz上にその座標変換がTF RViz pluginにより表示されているはずだ．
次のコマンドを実行してみよう．
```
$ rostopic echo /tf_static
$ rostopic hz /tf_static
```
最初のコマンドでは，一度だけ座標変換の内容が表示されるはずだ．
そして2つ目のコマンドでは，`new_message`と表示されつづけるはずである．
最初のコマンドを何度繰り返してもtopicの内容が一度だけ表示される．
これは/tf_staticが周期的にbroadcast(publish)されているのではなく，
それをlisten(subscribe)するものが現れたときに，
lesson1 nodeによって一度だけbroadcastされているからである．

次に`tf_lesson2.cpp`を実行する．
これは，tfをlistenする例であるが，
100Hz周期で現在時刻，1秒前，10秒前，最新利用可能時刻でのtfをlistenしてその成否を表示し，
更に1秒前，10秒前，最新利用時刻の指定で得た"world"から"base_link"への座標変換を
それぞれ赤色，緑色および青色のマーカーでRViz上に表示させるnodeである．
```
$ rosrun tf_lessons lesson2
```
実行すると最初だけ失敗するがすべての時刻においてtfがlistenできることがわかるだろう．

## Broadcast and Listen for dynamic(non-static) TF
前章で起動したRVizやroscore，nodeがそのままなら一度すべて終了させて，
roscore, RVizを起動しよう．
以下では"world"，"base_link"といった座標系の名前などを再利用するが，
Static, 非Staticなtfで同じ変換が混在する場合，RVizの表示などに不具合が生じる可能性があるようだ．

今回，lesson1 nodeは引数を変えて実行する．
'''
$ rosrun tf_lessons lesson1 non-static
'''
このとき，`tf2_ros::TransformBroadcaster`を用いて，/tf topicに座標変換をbroadcastしている．
一秒間に一回同じ変換をbroadcastすることを11回繰り返している．
RViz上でTF pluginにより座標変換が表示されるが，薄くなって消えていくのがわかるだろう．
非staticなtfは古いものは捨てられるtfの仕様のためかこのような表示が行われるようにこのpluginは実装されている．
(PluginをOn/Offすれば再び表示されるが...)

既に述べたようにtf listenerはデフォルトで過去10秒分のbroadcastされた座標変換の履歴を有しており，
それに基づき補間することで所望の座標変換を計算する．
一度lesson1 nodeを終了させて，lesson2 nodeを起動させたあとに再びlesson1 nodeを起動させてみよう．
lesson2 nodeの端末への出力から次のようなことが読み取れるはずだ．

最初に最新利用時刻"maybe-available", そして1秒前，10秒前の順にlistenが成功して，
時間が経つと1秒前のtfのlistenに失敗，そして10秒前のlistenに失敗する．
最新利用時刻のlistenはその後も可能であるが，その情報の時刻がlesson1 nodeによって
最後にbroadcastされたものであり，現在時刻"current-time"のlistenは常に失敗している．

これらから，tfのlisterが保持している10秒分のバッファの利用のされ方が理解できるであろう．

次はバッファから計算された座標変換が時間的に補間されたものであることを確認しよう．
lesson1 nodeを終了して，lesson3 nodeを起動する．
```
$ rosrun tf_lesson lesson3
```
lesson3 nodeでは"world"座標系に対して"base_link"座標系が1秒毎に36度だけ離散的に回転する座標変換を
broadcastしている．
lesson2 nodeも起動して座標変換をlistenした結果をマーカで可視化しよう．
このnodeによりRViz上で次のようなことが読み取れるはずだ．

最新利用時刻の座標変換を示す赤のマーカが"base_link"を指すtfによく追従しており，
1秒前の変換を示している緑のマーカがそのすぐ後ろを追従している．
10秒前の変換を示す青いマーカは時折消えることもあるが，"base_link"と一致しながら
連続的に動いているのがわかる．

lesson3 nodeは1Hzでtfをbroadcastしているので，10Hzでlistenしている
lesson2 nodeで赤いマーカは十分に追従できている．
10秒前の座標変換はlisterのバッファ的にギリギリの部分なのでlistenできたりできなかったりするために，
青いマーカは断続的に現れている．
また，緑と青のマーカは連続移動している．
"base_link"座標系は1秒に一回36度分回転しているだけなので正十角形の頂点を移動しているだけだが，
その頂点を移動している間の座標変換を時間的に補間することで
緑と青のマーカは十角形の外接円の円周上を移動している．

今回はlesson3 nodeが1Hzで/tfにbroadcast(publish)しているために
赤いマーカは"base_link"にきれいに追従したが，
tfのlistenはtopicのsubscribeと同じであるので，/tfの周波数が増えると
追従するが難しくなる．lesson3 nodeのコードを書き換えて試してもらいたい．

また余談ではあるが，lesson2 nodeでは
`tf2_ros::TransformBroadcaster`で座標変換をbroadcastしているが，
例えば`tf2_ros::StaticTransformBroadcaster`を使ってbroadcastすると，
/tf_staticをlistenするlistenerは座標系の補間を行わずに常に最新の座標変換のみを返す．
コードを書き換えてlesson2 nodeがpublishするmarkerの様子を見てみると理解が深まるだろう．

## static\_transform\_publisher node
`tf2_ros::StaticTransformBroadcaster`を利用したlesson1 nodeのように，
ある固定の座標変換を/tf_staticにbroadcastし続ける汎用的なnodeがtf2_ros packageにツールとして用意されており，
static_transform_publisherという．
次のように座標関係を指定することで実行できる．
```
$ rosrun tf2_ros static_transform_publisher 1 2 3 0 0 0 world base_link
```
指定している数値はxyzとrpyだがquaternionでもできる．
このnodeはtfで表現する座標系ツリーの中でせいぜい1つ2つの変換をbroadcastするのに用いることが多い．

static\_transform\_publisherは一つのnodeで一つの座標変換を扱っている．
staticなtfなので新しくsubscriberが現れた場合にしかbroadcastしないが，
同時に複数のstatic\_transform\_publisherを実行するとどうなるかlesson4.launchを実行して確かめてみよう．
```
$ roslaunch tf_lessons lesson4.launch
$ rostopic hz /tf_static # 別端末で
$ rostopic bw /tf_static # 別端末で
```
lesson4.launch では数珠つなぎにつながる座標変換を22個のstatic_transform_publisherで実行している．
RViz上に座標系が並んで表示されているのが見えるはずだ．
`rostopic hz` や `rostopic bw`の結果を見ると瞬間的に/tf_staticの周波数が増えたりネットワークを占める
バンド幅が瞬間的に増大していることがわかる．

static_transform_publisherでのbroadcastは周期的でないので問題になることはまず無いが，
無駄にnodeを実行しても仕方がないし，tfによってROSのtopicが飛ぶネットワークに
このように負荷がかかることがこの事例から理解できるだろう．
非staticなtfについてのこのような場合については次章でも確かめていく．

## Too many broadcasting node
ここからはroscoreとRVizを再び実行し直してから始める．

lesson5 nodeは`tf2_ros::TransformBroadcaster`を用いて非staticなtfをbroadcastするnode である．
引数を指定してある1つの座標変換をbroadcastするか複数の座標変換を同時にbroadcastするか設定することができる．
```
$ rosrun tf_lesson lesson5 onetf world base_link 0.5 # single tf by one node
$ rosrun tf_lesson lesson5 multitf world 20 0.5 # multiple(twenty) tf by one node
```
lesson5 nodeには100Hz周期でbroadcastさせている．これは`rostopic hz /tf`で調べられるだろう．

このlesson5 nodeを使って数珠つなぎの座標変換を複数のnodeを使ってbroadcastする場合と，
1つのnodeでbroadcastする場合の違いについて調べる．
複数のlesson5 nodeを同時に起動するために，lesson6.launchを用いる．
lesson6.launchではlesson5 nodeの引数に`onetf`を指定し，同時に22個起動して，
数珠つなぎの座標系変換をtfにbroadcastするように記述してある．
```
roslaunch tf_lessons lesson6.launch
```
lesson5 nodeでは100Hzで/tf topic にbroadcast(publish)しているはずだが，
`rostopic hz /tf`を実行すると/tfの周期が約2200Hzになっていることに気づくはずだ．

100Hzでpublishするnodeが独立に22個存在しているので結果的に約2200Hzのtopicになってしまっていることがわかる．
実際のセンサなどのnodeは100Hz以上で更新されることが多く，
考慮せずにtfをbroadcastするnodeを増やすと周波数は大きくなるこのような状況になってしまうだろう．
このような状況だと，各nodeのlisten自体が間に合わなくなったり，
割合低い周波数でbroadcastするSLAMのような他のnodeがあると，
その分のtfをlistenできなくなる場合が存在する
[参考](https://garaemon.github.io/ros/2014/12/31/ros.html).
lesson5 のコメント部分を外してtfに動きを付けて，同じtfの内容でも/tf topicの周波数の違いで
RVizの表示がどう異なるか試してみると良いだろう(ただこの辺は各々の環境の違いがでるかもしれない).

次はlesson5 nodeの引数に'multitf'を指定して，複数の座標変換を同時にtfにbroadcastしてみる．
```
rosrun tf_lessons lesson5 multitf world 22 0.5
```
lesson6.launchと同様に22個の"座標変換"がbroadcastされているがnodeは1つだけである．
ここでは一度に22個の座標変換をbroadcastしている．
`rostopic hz /tf`を実行すると，100Hzになっていることがわかる．
同様の結果が，より効率よく行われている．
`rostopic bw /tf`の結果も参考になるだろう．

実際のロボットのシステムでは複数の局地座標系を扱わざるを得ない．
そのために役立つURDFによる記述とrobot_state_publisher nodeについて次章にて述べる．

## URDF + joint_state_publisher + robot_state_publisher
一般にロボットのtfのbroadcastに関わる事例としては
* SLAM
* GPS
* コンパス
* IMU(velocity)
* カメラ
* 関節座標

が考えられる．
これらによる座標変換の中には固定であったり計算や観測の結果として座標変換をするものもあるが，
その殆どがURDFによる記述と`robot_state_publisher`で1つのtfにまとめてbroadcastすることができる．


## Keep one tf-tree
### Representative example of frames
/world, /odom, /base_link

### Connect fragments in ROSBAG data



