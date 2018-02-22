#ifndef WINDOW_H
#define WINDOW_H

#include <string>

#include <QWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLabel>

#include "config/config_params.hpp"
#include "config/config_file_reader.hpp"
#include "ccu_manager_gui.hpp"

class QPushButton;
class Window : public QWidget
{
    Q_OBJECT
public:
    explicit Window(std::string params_file, QWidget *parent = 0);
    virtual ~Window();

private slots:
    void goToStartClicked(bool checked);
    void goToMobidikClicked(bool checked);
    void goToElevatorClicked(bool checked);
    void enterElevatorClicked(bool checked);
    void exitElevatorClicked(bool checked);
    void pauseClicked(bool checked);
    void resumeClicked(bool checked);

private:
    QPushButton *go_to_start_btn_;
    QPushButton *go_to_mobidik_btn_;
    QPushButton *go_to_elevator_btn_;
    QPushButton *enter_elevator_btn_;
    QPushButton *exit_elevator_btn_;
    QPushButton *pause_btn_;
    QPushButton *resume_btn_;
    QPushButton *quit_btn_;

    QLabel *pose_label_;
    QLabel *command_label_;
    QLabel *progress_label_;

    CCUManager *ccu_manager_;
};

#endif
