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
#include <QColor>
#include <QScreen>

MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , ros2_node(ros2_node)
{
  main_widget = new QWidget(this);
  imageFrame = new QLabel(this);
  once = true;

  main_widget->setStyleSheet("background-color: #1F3347;");

  QHBoxLayout* main_layout = new QHBoxLayout;

  // Prepare Image
  QPixmap pixxer = showImage();
  imageFrame->setPixmap(pixxer);
  main_layout->addWidget(imageFrame);
  main_widget->setLayout(main_layout);

  setCentralWidget(main_widget);

  timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &MainGUI::updateImage);
  timer->start(200);

  showFullScreen();
  
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
 if(once) {
  once = false;
  windowSize = size();
  windowWidth = windowSize.width();
  windowHeight = windowSize.height();
 
 }
  
  QPixmap pixxer(windowWidth, windowHeight);
  pixxer.fill(QColor("#1F3347"));

  QImage im = showImage().toImage();
  QPainter painter(&pixxer);
  QFont font("Times New Roman", 14);
  painter.setFont(font);
  painter.setPen(Qt::white);

  // Define margins
  int leftMargin = windowWidth / 50;
  int rightMargin = windowWidth / 50;
  int topMargin = windowHeight / 30;
  int bottomMargin = windowHeight / 30;

  int adjustedWidth = pixxer.width() - 2*(leftMargin + rightMargin);
  int adjustedHeight = pixxer.height() - 2*(topMargin + bottomMargin);

  QString sonar_msg = QString::fromStdString("Sonar: " + ros2_node->getSonar());
  QString depth_msg = QString::fromStdString("Depth: " + std::to_string(ros2_node->getDepth()));
  QString imu_msg = QString::fromStdString("IMU: " + ros2_node->getIMU());
  QString orin_msg = QString::fromStdString("Orin Connection: " + ros2_node->getOrin());
  QString bag_msg = QString::fromStdString("bag: " + ros2_node->getBag());

  painter.drawImage(QRect(2*leftMargin, 2*topMargin, adjustedWidth, adjustedHeight), im);
  painter.drawText(leftMargin, topMargin-5, depth_msg);
  painter.drawText(pixxer.width()/2 - 40, topMargin-5, sonar_msg);
  painter.drawText(pixxer.width() - rightMargin - 120, topMargin-5, imu_msg);
  painter.drawText(leftMargin, pixxer.height()-bottomMargin/2, orin_msg);
  painter.drawText(pixxer.width() - rightMargin - 120, pixxer.height()-bottomMargin/2, bag_msg);

  imageFrame->setPixmap(pixxer);
  imageFrame->resize(pixxer.size());
}

