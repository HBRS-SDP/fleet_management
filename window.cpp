#include "window.h"

Window::Window(std::string params_file, QWidget *parent) :
    QWidget(parent)
{
    this->setFixedSize(500, 500);

    /////////////////
    // Label settings
    /////////////////
    pose_label_ = new QLabel("", this);
    pose_label_->setGeometry(10, 200, 400, 30);

    command_label_ = new QLabel("", this);
    command_label_->setGeometry(10, 240, 400, 30);

    progress_label_ = new QLabel("", this);
    progress_label_->setGeometry(10, 280, 400, 30);

    //////////////////
    // Button settings
    //////////////////
    go_to_start_btn_ = new QPushButton("Go to start", this);
    go_to_start_btn_->setGeometry(10, 10, 100, 30);

    go_to_mobidik_btn_ = new QPushButton("Go to MobiDik", this);
    go_to_mobidik_btn_->setGeometry(10, 50, 100, 30);

    go_to_elevator_btn_ = new QPushButton("Go to elevator", this);
    go_to_elevator_btn_->setGeometry(10, 90, 100, 30);

    enter_elevator_btn_ = new QPushButton("Enter elevator", this);
    enter_elevator_btn_->setGeometry(10, 130, 100, 30);

    exit_elevator_btn_ = new QPushButton("Exit elevator", this);
    exit_elevator_btn_->setGeometry(10, 170, 100, 30);

    pause_btn_ = new QPushButton("Pause", this);
    pause_btn_->setGeometry(120, 50, 100, 30);

    resume_btn_ = new QPushButton("Resume", this);
    resume_btn_->setGeometry(120, 90, 100, 30);

    quit_btn_ = new QPushButton("Quit", this);
    quit_btn_->setGeometry(120, 130, 100, 30);

    connect(go_to_start_btn_, SIGNAL (clicked(bool)), this, SLOT (goToStartClicked(bool)));
    connect(go_to_mobidik_btn_, SIGNAL (clicked(bool)), this, SLOT (goToMobidikClicked(bool)));
    connect(go_to_elevator_btn_, SIGNAL (clicked(bool)), this, SLOT (goToElevatorClicked(bool)));
    connect(enter_elevator_btn_, SIGNAL (clicked(bool)), this, SLOT (enterElevatorClicked(bool)));
    connect(exit_elevator_btn_, SIGNAL (clicked(bool)), this, SLOT (exitElevatorClicked(bool)));
    connect(pause_btn_, SIGNAL (clicked(bool)), this, SLOT (pauseClicked(bool)));
    connect(resume_btn_, SIGNAL (clicked(bool)), this, SLOT (resumeClicked(bool)));
    connect(quit_btn_, SIGNAL (clicked(bool)), QApplication::instance(), SLOT (quit()));

    ccu_manager_ = new CCUManager(ConfigFileReader::load(params_file), pose_label_, progress_label_);
}

Window::~Window()
{
    delete go_to_start_btn_;
    delete go_to_mobidik_btn_;
    delete go_to_elevator_btn_;
    delete enter_elevator_btn_;
    delete exit_elevator_btn_;
    delete pause_btn_;
    delete resume_btn_;
    delete quit_btn_;

    delete pose_label_;
    delete command_label_;
    delete progress_label_;

    delete ccu_manager_;
}

void Window::goToStartClicked(bool checked)
{
    std::string command = "START";
    ccu_manager_->sendGOTOCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    command_label_->setText(label);
}

void Window::goToMobidikClicked(bool checked)
{
    std::string command = "MOBIDIK";
    ccu_manager_->sendGOTOCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    command_label_->setText(label);
}

void Window::goToElevatorClicked(bool checked)
{
    std::string command = "ELEVATOR";
    ccu_manager_->sendGOTOCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    command_label_->setText(label);
}

void Window::enterElevatorClicked(bool checked)
{
    std::string command = "ENTER_ELEVATOR";
    ccu_manager_->sendElevatorCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    command_label_->setText(label);
}

void Window::exitElevatorClicked(bool checked)
{
    std::string command = "EXIT_ELEVATOR";
    ccu_manager_->sendElevatorCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    command_label_->setText(label);
}

void Window::pauseClicked(bool checked)
{
    std::string command = "PAUSE";
    ccu_manager_->sendCoordinationCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    command_label_->setText(label);
}

void Window::resumeClicked(bool checked)
{
    std::string command = "RESUME";
    ccu_manager_->sendCoordinationCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    command_label_->setText(label);
}
