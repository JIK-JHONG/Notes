#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QMessageBox>

class SimpleWindow : public QWidget {
public:
    SimpleWindow(QWidget *parent = nullptr) : QWidget(parent) {
        // 設定視窗標題和大小
        setWindowTitle("Qt 簡單介面");
        resize(300, 200);

        // 建立按鈕
        QPushButton *button = new QPushButton("點擊我", this);
        button->setGeometry(100, 80, 100, 30);

        // 連接按鈕點擊事件
        connect(button, &QPushButton::clicked, this, &SimpleWindow::showMessage);
    }

private:
    void showMessage() {
        QMessageBox::information(this, "訊息", "按鈕被點擊！");
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    SimpleWindow window;
    window.show();
    return app.exec();
}
