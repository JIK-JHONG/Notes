建立QT運作環境 於 MacOS (Apple Silicon)
-

**需先安裝brew 與 C++相關套件**

1. 步驟 1：安裝 Qt 環境


| Action | Code |  
|-------|-------|
| 使用 Homebrew 安裝 Qt | brew install qt|
| 檢查安裝位置(如確認有安裝，可略過） | brew --prefix qt |


通常 Homebrew 會將 Qt 安裝到 /opt/homebrew/opt/qt（Apple Silicon）或 /usr/local/opt/qt（Intel）。

2. 寫入環境變數：根據 Homebrew 安裝路徑，加入環境變數（可寫入 ~/.zshrc 或 ~/.bashrc）：


| Apple Silicon | Intel based |  
|-------|-------|
| export PATH="/opt/homebrew/opt/qt/bin:$PATH" | export PATH="/usr/local/opt/qt/bin:$PATH"

| export LDFLAGS="-L/opt/homebrew/opt/qt/lib" | export LDFLAGS="-L/usr/local/opt/qt/lib"
| export CPPFLAGS="-I/opt/homebrew/opt/qt/include" | export CPPFLAGS="-I/usr/local/opt/qt/include"
| export PKG_CONFIG_PATH="/opt/homebrew/opt/qt/lib/pkgconfig" | export PKG_CONFIG_PATH="/usr/local/opt/qt/lib/pkgconfig"



**Apple Silicon**

export PATH="/opt/homebrew/opt/qt/bin:$PATH"

export LDFLAGS="-L/opt/homebrew/opt/qt/lib" 

export CPPFLAGS="-I/opt/homebrew/opt/qt/include"

export PKG_CONFIG_PATH="/opt/homebrew/opt/qt/lib/pkgconfig"


**Intel based**

export PATH="/usr/local/opt/qt/bin:$PATH"

export LDFLAGS="-L/usr/local/opt/qt/lib"

export CPPFLAGS="-I/usr/local/opt/qt/include"

export PKG_CONFIG_PATH="/usr/local/opt/qt/lib/pkgconfig"



**使用 nano 編輯 與存檔**
nano ~/.zshrc

3. source ~/.zshrc  # 或 source ~/.bashrc
4. qmake --version

建立 Qt 專案
-
1. 建立專案資料夾 mkdir qt_demo or cmd + shfit + N (檔案管理員介面 / finder)

2. 建立 QT 的 main.cpp
3. qmake -project
4. nano qt_demo.pro (進入qt_demo.pro，並修改內容)

| Action | Description |  
|-------|-------|
|QT += widgets |-------|
|SOURCES += main.cpp |-------|
|CONFIG -= app_bundle |移除 macOS .app 形式，產生一般執行檔，如果要用.app 執行，可略|


編譯與執行
-

1. 產生 Makefile

| Action | Code |  
|-------|-------|
| 產生 Makefile | qmake qt_demo.pro|

2. 編譯

| Action | Code |  
|-------|-------|
| 編譯 | make|

3. 執行

| Action | Code |  
|-------|-------|
| 不保留.app | ./qt_demo|
| 保留.app | ./qt_demo.app/Contents/MacOS/qt_demo  or   open qt_demo.app |

