MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , ros2_node(ros2_node)
{
  // Initialization
  main_widget = new QWidget(this);
  imageFrame = new QLabel(this);
  orin = "Not Active";

  main_widget->setStyleSheet("background-color: #1F3347;");
  QHBoxLayout* main_layout = new QHBoxLayout;

  QPixmap pixxer = showImage();
  imageFrame->setPixmap(pixxer);
  main_layout->addWidget(imageFrame);
  main_widget->setLayout(main_layout);

  setCentralWidget(main_widget);

  count = 0;

  // Timer frequency increased to reduce CPU load
  timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &MainGUI::updateImage);
  timer->start(500);  // Adjusted to 500 ms or higher
    
  showFullScreen();
}

QPixmap MainGUI::showImage() {
  cv::Mat image = ros2_node->getRosMsg();
  
  if (image.empty()) {
    return QPixmap();
  }

  // Resize image to reduce processing load
  cv::resize(image, image, cv::Size(), 0.5, 0.5);
  
  QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(img.rgbSwapped());
  
  return pixmap;
}

void MainGUI::updateImage()
{
  if (count >= 500) {  // Increased interval for orin update
    orin = ros2_node->getOrin();
    count = 0;
  } else {
    count += 1;
  }

  QPixmap pixxer(800, 480);
  pixxer.fill(QColor("#1F3347"));

  QImage im = showImage().toImage();
  QPainter painter(&pixxer);
  QFont font("Times New Roman", 14);
  painter.setFont(font);
  painter.setPen(Qt::white);

  // Draw only the required information
  int leftMargin = 10, rightMargin = 10, topMargin = 30, bottomMargin = 30;
  int adjustedWidth = pixxer.width() - 2 * (leftMargin + rightMargin);
  int adjustedHeight = pixxer.height() - 2 * (topMargin + bottomMargin);

  QString sonar_msg = QString::fromStdString("Sonar: " + ros2_node->getSonar());
  QString depth_msg = QString::fromStdString("Depth: " + ros2_node->getDepth());
  QString imu_msg = QString::fromStdString("IMU: " + ros2_node->getIMU());
  QString orin_msg = QString::fromStdString("Orin Connection: " + orin);
  QString bag_msg = QString::fromStdString("bag: " + ros2_node->getBag());
  QString time_msg = QDateTime::currentDateTime().toString("hh:mm:ss");

  painter.drawImage(QRect(2 * leftMargin, 2 * topMargin, adjustedWidth, adjustedHeight), im);
  painter.drawText(leftMargin, topMargin - 5, depth_msg);
  painter.drawText(pixxer.width() / 2 - 40, topMargin - 5, time_msg);
  painter.drawText(pixxer.width() / 2 - 40, pixxer.height() - bottomMargin / 2, sonar_msg);
  painter.drawText(pixxer.width() - rightMargin - 120, topMargin - 5, imu_msg);
  painter.drawText(leftMargin, pixxer.height() - bottomMargin / 2, orin_msg);
  painter.drawText(pixxer.width() - rightMargin - 120, pixxer.height() - bottomMargin / 2, bag_msg);

  imageFrame->setPixmap(pixxer);
  imageFrame->resize(pixxer.size());
}

