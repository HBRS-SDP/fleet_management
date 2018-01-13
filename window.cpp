#include "window.h"

Window::Window(std::string params_file, QWidget *parent) :
    QWidget(parent)
{
    this->setFixedSize(500, 500);

    poseLabel = new QLabel("", this);
    poseLabel->setGeometry(10, 200, 400, 30);

    commandLabel = new QLabel("", this);
    commandLabel->setGeometry(10, 240, 400, 30);

    progressLabel = new QLabel("", this);
    progressLabel->setGeometry(10, 280, 400, 30);

    this->goToStartBtn = new QPushButton("Go to start", this);
    goToStartBtn->setGeometry(10, 10, 100, 30);

    this->goToMobidikBtn = new QPushButton("Go to MobiDik", this);
    goToMobidikBtn->setGeometry(10, 50, 100, 30);

    this->goToElevatorBtn = new QPushButton("Go to elevator", this);
    goToElevatorBtn->setGeometry(10, 90, 100, 30);

    this->enterElevatorBtn = new QPushButton("Enter elevator", this);
    enterElevatorBtn->setGeometry(10, 130, 100, 30);

    this->exitElevatorBtn = new QPushButton("Exit elevator", this);
    exitElevatorBtn->setGeometry(10, 170, 100, 30);

    this->pauseBtn = new QPushButton("Pause", this);
    pauseBtn->setGeometry(120, 50, 100, 30);

    this->resumeBtn = new QPushButton("Resume", this);
    resumeBtn->setGeometry(120, 90, 100, 30);

    this->quitBtn = new QPushButton("Quit", this);
    quitBtn->setGeometry(120, 130, 100, 30);

    connect(goToStartBtn, SIGNAL (clicked(bool)), this, SLOT (goToStartClicked(bool)));
    connect(goToMobidikBtn, SIGNAL (clicked(bool)), this, SLOT (goToMobidikClicked(bool)));
    connect(goToElevatorBtn, SIGNAL (clicked(bool)), this, SLOT (goToElevatorClicked(bool)));
    connect(enterElevatorBtn, SIGNAL (clicked(bool)), this, SLOT (enterElevatorClicked(bool)));
    connect(exitElevatorBtn, SIGNAL (clicked(bool)), this, SLOT (exitElevatorClicked(bool)));
    connect(pauseBtn, SIGNAL (clicked(bool)), this, SLOT (pauseClicked(bool)));
    connect(resumeBtn, SIGNAL (clicked(bool)), this, SLOT (resumeClicked(bool)));
    connect(quitBtn, SIGNAL (clicked(bool)), QApplication::instance(), SLOT (quit()));

    ccu_manager_ = new CCUManager(ConfigFileReader::load(params_file), poseLabel, progressLabel);
}

Window::~Window()
{
    delete goToStartBtn;
    delete goToMobidikBtn;
    delete goToElevatorBtn;
    delete enterElevatorBtn;
    delete exitElevatorBtn;
    delete pauseBtn;
    delete resumeBtn;
    delete quitBtn;

    delete poseLabel;
    delete commandLabel;
    delete progressLabel;

    delete ccu_manager_;
}

void Window::goToStartClicked(bool checked)
{
    std::string command = "START";
    ccu_manager_->sendGOTOCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    commandLabel->setText(label);
}

void Window::goToMobidikClicked(bool checked)
{
    std::string command = "MOBIDIK";
    ccu_manager_->sendGOTOCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    commandLabel->setText(label);
}

void Window::goToElevatorClicked(bool checked)
{
    std::string command = "ELEVATOR";
    ccu_manager_->sendGOTOCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    commandLabel->setText(label);
}

void Window::enterElevatorClicked(bool checked)
{
    std::string command = "ENTER_ELEVATOR";
    ccu_manager_->sendElevatorCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    commandLabel->setText(label);
}

void Window::exitElevatorClicked(bool checked)
{
    std::string command = "EXIT_ELEVATOR";
    ccu_manager_->sendElevatorCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    commandLabel->setText(label);
}

void Window::pauseClicked(bool checked)
{
    std::string command = "PAUSE";
    ccu_manager_->sendCoordinationCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    commandLabel->setText(label);
}

void Window::resumeClicked(bool checked)
{
    std::string command = "RESUME";
    ccu_manager_->sendCoordinationCommand(command);
    QString label = QString::fromStdString("Sending command: " + command);
    commandLabel->setText(label);
}
