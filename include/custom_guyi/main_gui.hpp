#pragma once

#include <QMainWindow>
#include <QWidget>
#include <QPushButton>

#include "custom_guyi/ros2node.hpp"
class MainGUI : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainGUI(const std::shared_ptr<Ros2Node>&  ros2_node, QWidget* parent = nullptr);
    ~MainGUI() override;
private:
    const std::shared_ptr<Ros2Node> ros2_node;

    QWidget* main_widget;
};
