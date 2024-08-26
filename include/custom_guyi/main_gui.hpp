#ifndef MAIN_GUI_HPP
#define MAIN_GUI_HPP

#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include <memory>
#include "custom_guyi/ros2node.hpp"

class MainGUI : public QMainWindow {
    Q_OBJECT

public:
    MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent = nullptr);
    ~MainGUI();

private slots:
    void updateImage();

private:
    QPixmap showImage();

    QWidget* main_widget;
    QLabel* imageFrame;
    QTimer* timer;
    std::shared_ptr<Ros2Node> ros2_node;

    QString orin;
    int count;
};

#endif // MAIN_GUI_HPP

