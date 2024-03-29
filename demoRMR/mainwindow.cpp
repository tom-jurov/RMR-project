#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
///TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU
#define SIM 1

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
#if SIM
    ipaddress="127.0.0.1";
#else
    ipaddress="192.168.1.14";
#endif
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;




    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);
    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);       
            }

            if(way_.size() > 0)
            {
                pero.setWidth(6);//hrubka pera -3pixely
                pero.setColor(Qt::blue);//farba je zelena
                painter.setPen(pero);
                for(int i=0; i < way_.size(); i++)
                {
                    int xp=rect.width()-(rect.width()/2 + 100*(way_[1].x - odom.getX())*sin(-odom.getHeading()) + 100*(way_[1].y - odom.getY())*cos(-odom.getHeading())) + rect.topLeft().x();
                    int yp=rect.height()-(rect.height()/2 + 100*(way_[1].x - odom.getX())*cos(-odom.getHeading()) - 100*(way_[1].y - odom.getY())*sin(-odom.getHeading())) + rect.topLeft().y();
                    int rx=rect.width()-(rect.width()/2) + rect.topLeft().x();
                    int ry=rect.height()-(rect.height()/2) + rect.topLeft().y();
                    if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    {
                        painter.drawEllipse(QPoint(xp, yp),2,2);
                        painter.drawEllipse(QPoint(rx, ry),10,10);
                    }
                }
            }

            pero.setWidth(3);//hrubka pera -3pixely
            painter.setBrush(QBrush());
            pero.setColor(Qt::yellow);//farba je zelena
            painter.setPen(pero);
            int xp=rect.width()-rect.width()/2+rect.topLeft().x(); //prepocet do obrazovky
            int yp=rect.height()-rect.height()/2+rect.topLeft().y();//prepocet do obrazovky
            if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                painter.drawEllipse(QPoint(xp, yp),270,270);

            if(edges.size() > 0)
            {
                pero.setWidth(6);//hrubka pera -3pixely
                pero.setColor(Qt::red);//farba je zelena
                painter.setPen(pero);
                for(int i=0; i < edges.size(); i++)
                {
                    int xp=rect.width()-(rect.width()/2 + 100*(edges[i].x - odom.getX())*sin(-odom.getHeading()) + 100*(edges[i].y - odom.getY())*cos(-odom.getHeading())) + rect.topLeft().x();
                    int yp=rect.height()-(rect.height()/2 + 100*(edges[i].x - odom.getX())*cos(-odom.getHeading()) - 100*(edges[i].y - odom.getY())*sin(-odom.getHeading())) + rect.topLeft().y();
                    if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                        painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }

            if(normals.size() > 0)
            {
                pero.setWidth(6);//hrubka pera -3pixely
                pero.setColor(Qt::yellow);//farba je zelena
                painter.setPen(pero);
                for(int i=0; i < normals.size(); i++)
                {
                    int xp=rect.width()-(rect.width()/2 + 100*(normals[i].x - odom.getX())*sin(-odom.getHeading()) + 100*(normals[i].y - odom.getY())*cos(-odom.getHeading())) + rect.topLeft().x();
                    int yp=rect.height()-(rect.height()/2 + 100*(normals[i].x - odom.getX())*cos(-odom.getHeading()) - 100*(normals[i].y - odom.getY())*sin(-odom.getHeading())) + rect.topLeft().y();
                    if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                        painter.drawEllipse(QPoint(xp, yp),2,2);
                }
            }

            pero.setWidth(6);//hrubka pera -3pixely
            painter.setBrush(QBrush());
            pero.setColor(Qt::magenta);//farba je zelena
            painter.setPen(pero);
            xp=rect.width()-(rect.width()/2 + 100*(follwed_point.x - odom.getX())*sin(-odom.getHeading()) + 100*(follwed_point.y - odom.getY())*cos(-odom.getHeading())) + rect.topLeft().x();
            yp=rect.height()-(rect.height()/2 + 100*(follwed_point.x - odom.getX())*cos(-odom.getHeading()) - 100*(follwed_point.y - odom.getY())*sin(-odom.getHeading())) + rect.topLeft().y();
            if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                painter.drawEllipse(QPoint(xp, yp),2,2);

        }
    }
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    if (copyOfLaserData.numberOfScans != 0)
    {
        //Real
        //goal_ = {3.84, 0.93};
        //goal_ = {0, 3};
        //Sim
        //goal_ = {4.45, 1.83};
        //goal_ = {4.45, 3.24};
        int dummy;
        controller.setGoal(goal_);
        way_ = local_nav.generateWaypoints(goal_, odom.getRobotState(),copyOfLaserData);
        controller.setPath(way_);

        /*edges = local_nav.findObstacleEdges(odom.getRobotState(), copyOfLaserData);
        normals = local_nav.findEdgeNormals(odom.getRobotState(), edges, 0.4);
        follwed_point = local_nav.findClosestAccessiblePoint(goal_, odom.getRobotState(), copyOfLaserData, normals, &dummy);*/
    }

    if(first_cycle_)
    {
        odom.setInitState(robotdata.EncoderLeft, robotdata.EncoderRight);
        map.setGyroStartAngle(robotdata.GyroAngle);
        first_cycle_ = false;
    }

    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky

//    if(forwardspeed==0 && rotationspeed!=0)
//        robot.setRotationSpeed(rotationspeed);
//    else if(forwardspeed!=0 && rotationspeed==0)
//        robot.setTranslationSpeed(forwardspeed);
//    else if((forwardspeed!=0 && rotationspeed!=0))
//        robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
//    else
//        robot.setTranslationSpeed(0);

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX
    odom.update(robotdata.EncoderLeft, robotdata.EncoderRight);
    map.setGyroAngle(robotdata.GyroAngle);
    if (start_)
    {
        controller.setCurrentPosition(odom);
        auto control_output = controller.controlStep();
        robot.setArcSpeed(control_output.speed, control_output.radius);
    }
    if(datacounter%5)
    {
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                ///ui->lineEdit_2->setText(QString::number(odom.getX()));
                ///ui->lineEdit_3->setText(QString::number(odom.getY()));
                ///ui->lineEdit_4->setText(QString::number(odom.getHeading()));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit uiValuesChanged(odom.getX(),odom.getY(),odom.getHeading());
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    if(odom.getAngularSpeed() < fabs(0.001))
    {
        map.update(laserData, odom);
    }

    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    //cout<<"W: " << cameraData.size().width<< endl;
    //cout<<"H: " << cameraData.size().height<< endl;
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    robot.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();
    odom.setWheelSeparation(0.23);
    //ziskanie joystickov
    instance = QJoysticks::getInstance();


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(500);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
robot.setRotationSpeed(-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setArcSpeed(0,0);
    start_=false;
}

void MainWindow::on_pushButton_8_clicked() //reset
{
    odom.setX(0);
    odom.setY(0);
    odom.setHeading(0);
}

void MainWindow::on_pushButton_10_clicked()
{
    auto waypoints = global_nav.generateWaypoints();
    controller.setPath(waypoints);
    start_ = true;
}

void MainWindow::on_pushButton_11_clicked()
{
    map.printMap();
}

void MainWindow::on_pushButton_12_clicked()
{
    TMapArea map;
    map_loader loader;
    loader.load_map("priestor.txt", map);
    auto occupancy_map = loader.createMap(map);
    auto bloated_occupancy_map = loader.createBloatedMap();
    double goal_x = 451.8;
    double goal_y = 339.11;
    global_nav = diff_drive::GlobalNav(bloated_occupancy_map, 0, 0, goal_x, goal_y);
    global_nav.floodFill();
    /*for (std::size_t y = 0; y<115; ++y)
    {
        for (std::size_t x = 0; x<115; ++x)
        {
            std::cout << occupancy_map[x][114-y];
        }
        std::cout << std::endl;
    }*/
    auto path = global_nav.getPath();
    for (const auto& p : path)
    {
        bloated_occupancy_map[p.x][p.y] = 3;
    }
    /*for (std::size_t y = 0; y<115; ++y)
    {
        for (std::size_t x = 0; x<115; ++x)
        {
            std::cout << bloated_occupancy_map[x][114-y];
        }
        std::cout << std::endl;
    }*/
}
void MainWindow::on_pushButton_13_clicked()
{
    start_ = true;
}

void MainWindow::on_pushButton_14_clicked()
{
    local_nav.first_edge_detected_ = true;
    goal_ ={0.0 , 0.0 };
}

void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}
