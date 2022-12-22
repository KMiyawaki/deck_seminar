# 2.5時間で分かる ROS サポートページ

このページでは[The Deck](https://thedeck.jp/)様にて開催予定の下記セミナー受講に必要な開発環境のインストール方法をご案内しています。

- [2.5時間で分かる！ROS対応ロボットの基礎]
- [2.5時間で「もっと」分かる！ROS対応ロボットの応用（近日イベントページ公開予定）]

セミナー受講予定の方のみご利用ください。  
なお、本開発環境で使用しているソフトウェアは世間一般で広く利用されているものです。また、構築した環境も担当講師が実際に動作確認してはいますが、残念ながら全てのPCにおいて安全な動作を保証することはできません。  
掲載された内容によって生じた損害等の一切の責任を負いかねますので、ご了承ください。不安がある場合は重要なファイル等のバックアップをされた上で実施されることをお勧めいたします。

- [2.5時間で分かる ROS サポートページ](#25時間で分かる-ros-サポートページ)
  - [環境構築に関する質問フォーム](#環境構築に関する質問フォーム)
  - [必要なソフトのインストール](#必要なソフトのインストール)
  - [Docker イメージのインストール](#docker-イメージのインストール)
  - [起動](#起動)
  - [終了](#終了)
  - [ROSの動作確認](#rosの動作確認)
    - [コマンドターミナルの起動とTurtlesim](#コマンドターミナルの起動とturtlesim)
    - [talkerとlistener](#talkerとlistener)
    - [ロボットシミュレータ](#ロボットシミュレータ)
  - [VSCodeとの連携](#vscodeとの連携)
  - [VNCクライアントの利用（オプション）](#vncクライアントの利用オプション)

## 環境構築に関する質問フォーム

開発環境構築で問題が生じた場合は[質問フォーム](https://forms.gle/cQJSa9ZCcbvkKJmr9)から必要事項をご入力の上送信してください。

セミナー受講予定の方のみを対象としています。それ以外の方はご遠慮ください。

## 必要なソフトのインストール

下記のソフトをインストールしてください。

1. [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)
   - インターネット上に日本語でのインストール方法についても解説があります。
   - [`WSL2`(Windows Subsystem for Linux 2)](https://learn.microsoft.com/ja-jp/windows/wsl/install)を併用する方法でインストールしてください。
2. [Visual Studio Code](https://azure.microsoft.com/ja-jp/products/visual-studio-code/)
   - 以降、`VSCode`と表記します。
3. WEBブラウザ
   - 種類は任意ですが、本セミナーでは[Chrome](https://www.google.com/intl/ja_jp/chrome/)を前提にご説明します。

## Docker イメージのインストール

Wifi通信の良好な環境にて実施してください。

[ros_lecture_melodic_202302-main.zip](https://github.com/KMiyawaki/ros_lecture_melodic_202302/archive/refs/heads/main.zip)をダウンロードし展開してください。ここでは`C:\Users\[ユーザ名]\Documents\ros_lecture_melodic_202302-main`に展開したとします。

なお、`[ユーザ名]`には`Windows`のログインユーザ名が入ります。以降も同様です。

展開後のフォルダには以下のファイルが含まれています。

```text
    docker-compose.yml
    README.md
    images # 説明文書の画像
```

まず[Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)が**起動していることを必ず確認してください**。

以下の画面のようにWindowsのタスクトレイに`Docker Desktop for Windows`のアイコンが表示されていることを確認し、次の手順に移ってください。  
アイコンの表示が無ければ、`Windows`の[アプリ検索](https://support.microsoft.com/ja-jp/windows/%E3%81%99%E3%81%B9%E3%81%A6%E3%81%AE%E3%82%A2%E3%83%97%E3%83%AA%E3%81%A8%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%A0%E3%82%92%E6%A4%9C%E7%B4%A2%E3%81%99%E3%82%8B-cadb9c4b-459d-dfcb-2964-14aac1d7d964)で`Docker Desktop for Windows`を検索して実行してください。

![2022-12-11_102434.png](./images/2022-12-11_102434.png)

`Docker Desktop for Windows`の起動を確認できたら`Windows`のコマンドプロンプトを起動し、次のコマンドを入力してください。  
なお、ファイルパスの`\`バックスラッシュは![2022-12-11_102434.png](./images/yen.png)（円マーク）を意味しています。

```cmd
cd C:\Users\[ユーザ名]\Documents\ros_lecture_melodic_202302-main
docker-compose pull
```

以下の画面のように環境のダウンロードが始まります。

![2022-12-11_154539.png](./images/2022-12-11_154539.png)

終了したら次のコマンドを入力してください。

```cmd
docker images
```

次のように`melodic_202302`のイメージがダウンロードされていれば成功です。  
`76e6332fcd35   22 minutes ago   3.81GB`の部分は多少異なる可能性があります。

```cmd
REPOSITORY                TAG                IMAGE ID       CREATED          SIZE
kmiyawaki20/ros_lecture   melodic_202302   76e6332fcd35   22 minutes ago   3.81GB
```

## 起動

以下の画面のようにWindowsのタスクトレイに`Docker Desktop for Windows`のアイコンが表示されていることを確認し、次の手順に移ってください。

`Docker Desktop for Windows`が起動されていなければ、起動してから次の手順に移ってください。

![2022-12-11_102434.png](./images/2022-12-11_102434.png)

`Docker Desktop for Windows`の起動を確認できたら、次のコマンドを入力してください。

```cmd
cd C:\Users\[ユーザ名]\Documents\ros_lecture_melodic_202302-main
docker-compose up
```

次のようなメッセージが出力されます。

このコマンドを起動したターミナルは作業終了までは閉じないでください。

```cmd
 - Network ros_lecture_melodic_202302-main_default  Created                                                      0.8s
 - Volume "ros_lecture_melodic_202302-main_ubuntu"  Created                                                      0.0s
 - Container ros_lecture_melodic_202302             Created                                                      8.7s
Attaching to ros_lecture_melodic_202302
・・・
ros_lecture_melodic_202302  | **********************************************
ros_lecture_melodic_202302  | * Open 'http://127.0.0.1:6080/vnc.html'      *
ros_lecture_melodic_202302  | * Or access '127.0.0.0:5901' via VNC viewer. *
ros_lecture_melodic_202302  | **********************************************
```

任意のWEBブラウザで[http://127.0.0.1:6080/vnc.html](http://127.0.0.1:6080/vnc.html)に接続してください。  
次のような画面が出ますので、`Connect`を押してください。パスワードは不要です。

![2022-12-22_103632.png](./images/2022-12-22_103632.png)

次のように`Linux`の`GUI`が表示されれば成功です。

![2022-12-22_104159.png](./images/2022-12-22_104159.png)

もしも、`Linux`のデスクトップ画面が大きすぎてブラウザに入りきらない場合は`docker-compose.yml`をメモ帳で次のように変更してください。

```yaml
・・・
    environment:
      - USER=ubuntu
      - RESOLUTION=1024x600 # RESOLUTION=1280x800の数値を小さくする
      - TZ=Asia/Tokyo
    tty: true
・・・
```

編集後、以下の[終了](#終了)で一旦終了してから再度[起動](#起動)をしてみてください。

## 終了

WEBブラウザを閉じ、起動コマンドを実行したコマンドプロンプトで`Ctrl`キーと`C`キーを同時に押してください。

もし、終了しなければ２，３回`Ctrl`キーと`C`キーを同時に押してください。それでも終了しない場合は起動コマンドを実行したコマンドプロンプトを閉じてください。

## ROSの動作確認

練習のために、できれば一度PCを再起動し、上記の「[起動](#起動)」をもう一度実施してみてから次の手順に移ってください。

### コマンドターミナルの起動とTurtlesim

WEBブラウザで表示されている`Linux`の`GUI`（以降単に`Linux`と表記します）のコマンドターミナルを起動してください。  
画面左下のアイコンをクリックし、`System Tools`->`LXTerminal`をクリックします。

![2022-12-22_104451.png](./images/2022-12-22_104451.png)

コマンドターミナルで`roscore`を入力してエンターキーを押してください。実行結果は次の通りです。

![2022-12-11_110107.png](./images/2022-12-11_110107.png)

このコマンドターミナルは動作確認終了までは閉じないでください。

再度`System Tools`->`LXTerminal`をクリックして、もう一つコマンドターミナルを開き、次のコマンドを実行してください。

```shell
rosrun turtlesim turtlesim_node
```

なお、WEBブラウザ上での`Linux`に対し`Windows`側でコピーしたテキストを単純な方法ではペーストすることはできません。`Linux`->`Windows`も不可能です。  
`Linux`->`Linux`へのコピー＆ペーストは可能です。

`Windows`->`Linux`へのコピー＆ペーストをする場合は画面右上のクリップボードアイコンをクリックし、そこにテキストをペーストしてください。  
その後、`Linux`側で右クリックからペーストを行ってください。

![2022-12-22_110016.png](./images/2022-12-22_110016.png)

`Linux`->`Windows`の場合は`Linux`側でコピーしたテキストが自動的にクリップボードに表示されますので、それを利用してください。

`rosrun turtlesim turtlesim_node`を実行すると次の画面のように青いウィンドウと亀のキャラクタが表示されます。キャラクタは表示のたびに変わります。

![2022-12-11_110833.png](./images/2022-12-11_110833.png)

確認が終わったら`rosrun turtlesim turtlesim_node`を起動したターミナルで`Ctrl+C`を押して終了させてください。

### talkerとlistener

画面左下のアイコンをクリックし、`System Tools`->`LXTerminal`をクリックして、新しいコマンドターミナルを起動し、次のコマンドを実行してください。

```shell
rosrun beginner_tutorials talker.py
# 以下実行結果
[INFO] [1671266060.656336]: hello world 1671266060.66
[INFO] [1671266060.756805]: hello world 1671266060.76
[INFO] [1671266060.856629]: hello world 1671266060.86
・・・
```

もう一つコマンドターミナルを起動し次のコマンドを実行してください。

```shell
rosrun beginner_tutorials listener.py 
# 以下実行結果
[INFO] [1671266176.099021]: /listener_18191_1671266175889I heard hello world 1671266176.1
[INFO] [1671266176.197755]: /listener_18191_1671266175889I heard hello world 1671266176.2
[INFO] [1671266176.297876]: /listener_18191_1671266175889I heard hello world 1671266176.3
・・・
```

もう一つコマンドターミナルを起動し次のコマンドを実行してください。

```shell
rqt_graph
```

次のような画面が出たら成功です。すべてのターミナルを`Ctrl+C`で終了させてください。

![2022-12-22_111234.png](./images/2022-12-22_111234.png)

### ロボットシミュレータ

前項と同じ要領で`LXTerminal`を起動し、次のコマンドを入力してください。

```shell
roslaunch oit_navigation_minibot_light_01 stage_navigation.launch
```

シミュレータが起動しますので、`2D Nav Goal`ボタンを押して、地図上の適当な場所をクリックしてください。  
ロボットが経路を計画し、自律移動します。場所によってはたどりつけないこともありますが、問題はありません。
目的地においてマウスドラッグすることで到着時の方向を指定することができます。

以下の動画のデスクトップの背景やウィンドウのタイトルバーなどが、配布している開発環境で実行した場合は異なることがありますが、問題はありません。

![AGDRec_20221211_160113.gif](./images/AGDRec_20221211_160113.gif)

シミュレータは起動コマンドを入力した`LXTerminal`で`Ctrl+C`を押して終了させてください。

## VSCodeとの連携

`VSCode`の拡張機能から`Docker`、`Dev Containers`、`Remote Development`を選択してインストールしてください。

![2022-12-17_170506.png](./images/2022-12-17_170506.png)

![2022-12-22_150325.png](./images/2022-12-22_150325.png)

![2022-12-22_151530.png](./images/2022-12-22_151530.png)

インストール後は`VSCode`を一旦再起動してください。

[起動](#起動)の項目で実施した`docker-compose up`が正常に実行され、WEBブラウザで[http://127.0.0.1:6080/vnc.html](http://127.0.0.1:6080/vnc.html)に接続可能なことを確認してから、次の手順に移ってください。

`VSCode`のリモートエクスプローラをクリックして`Containers`をクリックしてください。  

![2022-12-17_170846.png](./images/2022-12-17_170846.png)

`ros_lectures_melodic_202302`があるはずですので、それをクリックしてください。  
さらに、`Attach to Container`をクリックしてください。

![2022-12-17_170946.png](./images/2022-12-17_170946.png)

次のようなダイアログが出た場合は`Got It`をクリックしてください。

![2022-12-17_171023.png](./images/2022-12-17_171023.png)

`VSCode`が`Linux`に接続します。以下のような表示が出ることがありますが、問題はありません。「今後表示しない」や「X（バツボタン）」をクリックして次に進んでください。

![2022-12-22_113707.png](./images/2022-12-22_113707.png)

![2022-12-22_093913.png](./images/2022-12-22_093913.png)

左下の方に`Container kmiyawaki20/ros_lectures...`という表示が出ていることを確認してください。  
左側のフォルダを開くアイコンをクリックし、`/home/ubuntu/catkin_ws/src`を選択してください。

![2022-12-17_171129.png](./images/2022-12-17_171129.png)

`/home/ubuntu/catkin_ws/src/beginner_tutorials/scripts/listener.py`をダブルクリックし、エディタで表示できることを確認してください。

![2022-12-17_171223.png](./images/2022-12-17_171223.png)

`Ctrl+Shift+@`を同時に押してください。`VSCode`上でコマンドターミナルが開きます。  
`Ctrl+Shift+@`の同時押しを繰り返せばターミナルは複数個開くことができます。  
不要なターミナルは`exit`コマンド、もしくはゴミ箱のボタンを押すことで閉じることができます。


練習のために、`roscore`、`rosrun beginner_tutorials talker.py`、`rosrun beginner_tutorials listener.py`を`VSCode`から実行してみてください。

なお、シミュレータ`roslaunch oit_navigation_minibot_light_01 stage_navigation.launch`を含むGUIのソフトを`VSCode`から実行することはできません。

基本的な開発の流れとしては次のようになります。

1. WEBブラウザ上の`Linux`でシミュレータを起動する。
2. `VSCode`を`Linux`に接続し、`Python`コード編集を行う。
3. `VSCode`のターミナルから`Python`コードを実行する。

## VNCクライアントの利用（オプション）

前項までの環境でセミナーの受講は可能ですが、VNCクライアントソフトを入れることで少し作業がしやすくなります。

1. コピー＆ペーストがダイレクトにできるようになる。
2. `Linux`側ターミナルで`Shift+Ctrl+T`によりタブを増やすことができる。
   - １つのターミナルウィンドウに複数のコマンド実行用のタブを作成できるのでデスクトップ画面がすっきりする。

VNCクライアントソフトには様々なものがありますが、ここでは[Real VNC](https://www.realvnc.com/en/connect/download/viewer/)を使います。リンクからダウンロードし、インストールしてください。

インストール後、`VNC Viewer`を実行し、接続先に`127.0.0.1:5901`を入力してエンターを押してください。

`Unencrypted Connection`の警告が出ますが`Don't warn me`にチェックを入れて`Continue`をクリックしてください。

![2022-12-22_160731.png](./images/2022-12-22_160731.png)

WEBブラウザのときと同じように`Linux`のデスクトップにアクセスできます。  
ターミナルソフトを起動して`Shift+Ctrl+T`を押すと、ターミナル内でタブを作成することができます。
ターミナルに`Windows`側からコピーしたコマンドを右クリックでペーストできます。

![2022-12-22_161539.png](./images/2022-12-22_161539.png)

`VNC Viewer`で接続した`Linux`デスクトップ表示の色数が少ない場合は、一旦`Linux`を「X」（バツボタン）で閉じ、`VNC Viewer`の下記画面で`127.0.0.1:5901`の接続先アイコンを右クリックし、`Properties`の画面を開いてください。

![2022-12-22_161354.png](./images/2022-12-22_161354.png)

`Options`の`Picture quality`を`HIGH`にして`OK`を押してください。  

![2022-12-22_161431.png](./images/2022-12-22_161431.png)

設定後、`127.0.0.1:5901`の接続先アイコンをダブルクリックして再接続してください。
