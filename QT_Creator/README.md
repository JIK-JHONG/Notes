QT Creator 使用說明 於 MacOS (Apple Silicon)
-

**需先安裝brew 與 C++相關套件**

1. 步驟 1：安裝 QT Creator 環境 與 執行


| Action | Code |  
|-------|-------|
| 使用 Homebrew 安裝 QT Creator | brew install --cask qt-creator|
| 執行QT Creator | open -a "Qt Creator" |


是專案類型，可以建立有介面的QT 或是 沒有介面的QT，這個只是一個IDE。


> [!CAUTION]
> 注意：使用QT Creator執行編譯好的C++ program時，**有機會**會出現**記憶體錯誤**（IDE＋Apple Silicon問題，尤其透過 brew 安裝的QT套件）。\
可以使用 terimal 進行執行 
>
> 


![介面](https://github.com/JIK-JHONG/Notes/blob/main/QT_Creator/images/QT_workspace.jpeg)


2. 專案結構

src/
-
├── main.cpp          // 程式入口

└── mainwindow.cpp    // 主要視窗的實現

└── mainwindow.h      // 主要視窗的頭文件

ui/
-
└── mainwindow.ui   // 主要視窗的 UI 設計檔案

# C++
**如果變動，建議手動執行，生成 uic mainwindow.ui -o ui_mainwindow.h**

> [!NOTE]
> uic mainwindow.ui -o ui_mainwindow.h


# python 

**pyside6-uic form.ui -o ui_form.py**

> [!NOTE]
> pyside6-uic form.ui -o ui_form.py

build/
-
└── debug/           // 用於存放調試版本的編譯檔案

└── release/         // 用於存放釋出版本的編譯檔案

resources/
-
├── images/

│   └── icon.png

translations/
-
├── en.ts   // 英文翻譯檔案

└── zh.ts   // 中文翻譯檔案



宣告元件
-

1. 一般來說，在UI中定義好的元件，可以於 **mainwindow.h** 中進行「指標(pointor)」宣告，

如：

在**mainwindow.ui**定義一個UI元件，<widget class="QLabel" name="element_QLabel">，
可以在**mainwindow.h** 中進行「指標(pointor)」private:內宣告，QLabel *element_QLabel,

於**mainwindow.cpp**進行綁定功能（含監聽、實作等）：

# 設定數值：
ui->element_QLabel->setText(QString::number(<input_value>)); 
# 監聽綁定：
這邊以QSlider爲範例。

connect(ui->element_QSlider, &QSlider::valueChanged, this, &MainWindow::<EVENT>);

數值改變就執行<EVENT>


這邊以QBtn爲範例。

connect(ui->element_Btn, &QPushButton::clicked, this, &MainWindow::<EVENT>);

點擊就執行<EVENT>

...

**除錯 qDebug()，用法類似cout<<**

qDebug() << "msg = " << <val> ;

2. 常用元件：


| 元件 | 類型 | 寫入 |  取值 |   
|-------|-------|-------|-------|
| QLabel | 標籤 | setText() |  text() |  
| QPushButton | 按鈕 | setText() |  text() |  
| QLineEdit | 單行文字 | setText() |  text() |  
| QTextEdit | 多行文字 | setPlainText() / setHtml() |  toPlainText() / toHtml() |
| QComboBox | 下拉選單 | addItem() / addItems() |  currentText()  / currentIndex() |  
| QRadioButton | 單選按鈕 | setChecked() |  isChecked() |  
| QCheckBox | 檢核 | setChecked() |  isChecked() | 
| QSlider | 滑動條(水平/垂直) | setValue() / setRange(int<min>,int<max>) |  value() |
| QSpinBox | 數字輸入框 | setValue() / setRange(int<min>,int<max>) |  value() |
| QProgressBar | 顯示進度條 | setValue() |  value() | 
| QTableWidget | 顯示表格資料 | setItem() |  item() / selectedItems() | 
| QListWidget | 顯示列表 | addItem() |  currentItem() / selectedItems() | 
| QDateEdit | 日期選擇器 | setDate() |  date()| 


建置(Build)
-

每次變更完，都需要進行建置。

qmake
-

當有變動到 <project>.pro 或是 UI有變更都要執行。

C++ based

qmake <專案名稱>.pro -o Makefile

make


執行
-

**C++ based**

MacOS 通常會打包為一個 .app

./<專案名稱>.app/Contents/MacOS/<專案名稱>

**Python based**

python widget.py




