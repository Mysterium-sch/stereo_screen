#include "custom_guyi/main_gui.hpp"
#include <opencv2/opencv.hpp>
#include <QPushButton>
#include <QBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QPainter>

MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , ros2_node(ros2_node)
{
  main_widget = new QWidget(this);
  imageFrame = new QLabel(this);

  main_widget->setStyleSheet("background-color: #1F3347;");

  QHBoxLayout* main_layout = new QHBoxLayout;
  main_layout->setSpacing(20);
  main_layout->setMargin(20);

  // Prepare Image
  QPixmap pixxer = showImage();
  QPainter painter(&pixxer);
  QFont font("Times New Roman", 12);
  painter.setFont(font);
  painter.setPen(Qt::black);
  painter.drawText(10, 20, "Your Text Here");

  imageFrame->setPixmap(pixxer);
  main_layout->addWidget(imageFrame);
  main_widget->setLayout(main_layout);

  setCentralWidget(main_widget);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainGUI::updateImage);
    timer->start(10);

}

MainGUI::~MainGUI()
{
}

QPixmap MainGUI::showImage() {
  cv::Mat image = ros2_node->getRosMsg();

  QImage img(image.data, image.cols, image.rows, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(img);

  return pixmap;
}

void MainGUI::updateImage()
{

    QPixmap pixmap = showImage();

    imageFrame->setPixmap(pixmap);
    imageFrame->resize(pixmap.size());
}