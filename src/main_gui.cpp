#include "custom_guyi/main_gui.hpp"

#include <QPushButton>
#include <QBoxLayout>

MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
  : QMainWindow(parent)
  , ros2_node(ros2_node)
{
  main_widget = new QWidget(this);
  main_widget->setStyleSheet("background-color: #1F3347;");

  QHBoxLayout* main_layout = new QHBoxLayout;
  main_layout->setSpacing(20);
  main_layout->setMargin(20);
  
  main_widget->setLayout(main_layout);

  setCentralWidget(main_widget);
}

MainGUI::~MainGUI()
{
}
