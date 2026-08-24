#include "LidarReader.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <algorithm>
using namespace std;
//FULL_RECONDTRSNTED
LidarReader::LidarReader(mySerial* serial,mySerial* pico) : serial(serial), pico(pico), running(true) {
    uartThread = std::thread(&LidarReader::uartReaderThread, this);
}

LidarReader::~LidarReader() {
    running = false;
    if (uartThread.joinable())
        uartThread.join();
}
void LidarReader::uartReaderThread() {
    unsigned char buf[4096];
    std::vector<unsigned char> buffer; // Локальный буфер для накопления данных
    unsigned char buf1[4096];
    std::vector<unsigned char> buffer1; // Локальный буфер для накопления данных
    long long mo=pow(10,11)+7;
    counter=0;
    int packet_len=0;
    int data_len=0;
    int scan_freqx20=0;
    int start_anglex100=0;
    unsigned int dist_mm;
    float angle_deg;
    int angleKey;
    int quality;
    int coun=1;
    int last;
    //FILE *fin;
    //fin = fopen( "testrpi.txt", "w+" );
    while (running) {
        int n = serial->ReceiveNonBlocking(buf, sizeof(buf));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        buffer.insert(buffer.end(), buf, buf + n); 
        size_t i = 0;
        size_t i1= 0;
        int n1 = pico->ReceiveNonBlocking(buf1, sizeof(buf1));
        if (n1 <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        buffer1.insert(buffer1.end(), buf1, buf1 + n1);
        //cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;
        //for(int g=0;g<buffer.size();++g) fprintf(fin,"%c",buffer[g]);
        float mi_d=7.6;
        float mi_a=0; 
        while (i < buffer.size()-6) {
            // Ищем маркер начала луча
            if (buffer[i] != 0xAA||
		buffer[i+3] != 01 ||
		buffer[i+4] != 0x61 ||
                buffer[i+5] != 0xAD) {
                ++i;

                continue;
            }
	    if (i + 20 > buffer.size()) break; // Ждём следующего чтения
            packet_len=(unsigned int) buffer[i+2] | (buffer[i+1] << 8);
            data_len=(unsigned int) buffer[i+7] | (buffer[i+6] << 8);
	    //if(packet_len-data_len!=3) continue;
            scan_freqx20=(int)buffer[i+8];
            start_anglex100=(unsigned int) buffer[i+12] | (buffer[i+11] << 8);
            float coff=(67.5/(data_len-5));
            float start_angle=((start_anglex100/100)%360+270)%360;
	    //float koff = 246752;
            if(i+packet_len>buffer.size()) break;
            /*
	    bool corrupted = false;
            for (size_t j = i + 1; j < i + 12; ++j) {
                if (buffer[j] == 0xAA) {
                    corrupted = true;
                    break;
                }
            }
            if (corrupted) {
                i += 1;
                continue;
            }*/
	    //cout<<"pack_len: "<<packet_len<<endl<<" data_len: "<<data_len<<endl<<"start angle: "<<(start_anglex100/100)%360<<endl<<" scan_freqx20: "<<scan_freqx20<<endl;
    	    //int dist_mm;
            //int dist_mmx4;
            //if(start_angle==0) std::cout << "\033[H\033[2J\033[3J"; //clearscreen
//	    if(start_angle==0) std::cout << "\033[H";
            for(int f=3;f<data_len-5;f+=3){
		//cout<<(int)buffer[i+f]<<" ";
                angle_deg=start_angle+coff*(f/3);
                while(angle_deg>360) angle_deg-=360;
                //angle_deg=round(angle_deg);
                angleKey=angle_deg*100;//round(angle_deg*10);
                quality=buffer[i+f+13-3];
                dist_mm= (buffer[i+f+13-2])<<8 | buffer[i+f+13-1]; //(buffer[i+f+13-3]   (buffer[i+f+13-1]<<8))/4;
		if(quality<=100||dist_mm<=0.25) continue;
                //if(start_angle==0) printf("%02x %02x %02x ",buffer[i+f+13-3],buffer[i+f+13-2],buffer[i+f+13-1]); 
                //if(dist_mm<0)continue;
                LidarPoint p;

                //cout<<"dist!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: "<<dist_mm/4;
                //printf(" %02x %02x \n",buffer[i+f+13-2],buffer[i+f+13-1]);
                p.angle=angle_deg*M_PI/180.;
                p.distance=((double)dist_mm/4000.+latestScan[angleKey].distance)/2;
                std::lock_guard<std::mutex> lock(scanMutex);
                counter++;
                if(counter>mo)counter-=mo;
                time[angleKey]=counter;
                if(p.distance<2) latestScan[angleKey] = p;
              /*  if(p.distance<mi_d&&p.distance>0.20){
			mi_d=p.distance;
			mi_a=angle_deg;
	   	}*/
	    }
            //cout<<endl;
            i+=packet_len;
	    /*if(start_angle==0){
		cout << "\033[H\033[2J\033[3J";
		cout<<"min_d: "<<mi_d<<" min_a: "<<mi_a<<endl;
	        mi_d=7.6;
                mi_a=0;
	    }*/
       }
       while (i1 + 7 <= buffer1.size()) {
            if (buffer1[i1] != 0xAA) { ++i1; continue; } // ищем маркер луча
            float angle_deg1;
            short dist_mm1;
            std::memcpy(&angle_deg1, &buffer1[i1 + 1], sizeof(float));
            std::memcpy(&dist_mm1, &buffer1[i1 + 5], sizeof(short));
            if(angle_deg1==361)left=dist_mm1/1000.;
            if(angle_deg1==362)right=dist_mm1/1000.; 
            i1 += 7; // следующий луч
       }
        //if (i > 0) buffer.erase(buffer.begin(), buffer.begin() + i-3);
        //if (i1 > 0) buffer1.erase(buffer1.begin(), buffer1.begin() + i1-3);

/*
            // Всё ок, парсим угол и дистанцию
            float angle_deg;
            short dist_mm;
            memcpy(&angle_deg, &buffer[i + 1], sizeof(float));
            memcpy(&dist_mm, &buffer[i + 5], sizeof(short));
            if(angle_deg==361) left=dist_mm/1000;
            if(angle_deg==362) right=dist_mm/1000;
            if (dist_mm > 0 && angle_deg >= 0 && angle_deg <= 360) {
                LidarPoint p;
                p.angle = angle_deg * M_PI / 180.0;
                p.distance = dist_mm / 1000.0;

                int angleKey = static_cast<int>(angle_deg * 100); // ключ по углу
                std::lock_guard<std::mutex> lock(scanMutex);
                counter++;
                if(counter>mo)counter-=mo;
                time[angleKey]=counter;
                if(p.distance<7.5) latestScan[angleKey] = p;

            }

            i += 7; // Переходим к следующему возможному лучу
        }

        // Оставляем в буфере неполные данные для следующего прохода
        if (i > 0) buffer.erase(buffer.begin(), buffer.begin() + i);
    */
    }
    //fclose(fin);
}


/*
void LidarReader::uartReaderThread() {
    unsigned char buf[4096];

    std::vector<unsigned char> buffer; // накопительный буфер для пакетов

    while (running) {
        int n = serial->ReceiveNonBlocking(buf, sizeof(buf));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        buffer.insert(buffer.end(), buf, buf + n);

        // разбираем накопленный буфер
        size_t i = 0;
        while (i + 7 <= buffer.size()) {
            if (buffer[i] != 0xAA) { ++i; continue; } // ищем маркер луча

            float angle_deg;
            short dist_mm;
            std::memcpy(&angle_deg, &buffer[i + 1], sizeof(float));
            std::memcpy(&dist_mm, &buffer[i + 5], sizeof(short));

            if (dist_mm > 0 && angle_deg >= 0 && angle_deg <= 360) {
                LidarPoint p;
                p.angle = angle_deg * M_PI / 180.0;
                p.distance = dist_mm / 1000.0;

                int angleKey = static_cast<int>(angle_deg * 100); // индекс по углу
                std::lock_guard<std::mutex> lock(scanMutex);
                latestScan[angleKey] = p;
            }

            i += 7; // следующий луч
        }

        if (i > 0)
            buffer.erase(buffer.begin(), buffer.begin() + i);
    }
}*/

bool LidarReader::readScan(std::vector<LidarPoint>& scan) {
    std::lock_guard<std::mutex> lock(scanMutex);
    scan.clear();
    int coun=0;
    cout<<"_________________________________________________________________________________________________________SCAN: "<<latestScan.size()<<endl;
    for (auto& kv : latestScan){
//          scan.push_back(kv.second);
        if(time[kv.first]-counter>-2000){ 
          scan.push_back(kv.second);
          cout<<"ANGLE:"<<kv.first<<" DIST: "<<kv.second.distance<<" TIME: "<<time[kv.first]<<endl;
          coun++;
        }
    }
    std::cout<<"point_cloude:"<<scan.size()<<" CHECK: "<<coun<<endl;
    return !scan.empty();
}

double LidarReader::getright() { return right; }
double LidarReader::getleft() { return left; } 
/*
bool LidarReader::readScan(std::vector<LidarPoint>& scan) {
    scan.clear();
    if (!serial || !serial->IsOpen()) return false;

    unsigned char temp[2048];
    int n = serial->ReceiveNonBlocking(temp, sizeof(temp));
    if (n <= 0) return false;

    buffer.insert(buffer.end(), temp, temp + n);

    const size_t PACKET_SIZE = 1 + 4 + 2; // 0xAA + float(angle) + short(dist)
    size_t i = 0;

    while (i + PACKET_SIZE <= buffer.size()) {
        if (buffer[i] != 0xAA) { ++i; continue; }

        float angle_deg;
        short dist_mm;
        std::memcpy(&angle_deg, &buffer[i + 1], sizeof(float));
        std::memcpy(&dist_mm, &buffer[i + 5], sizeof(short));

        if (dist_mm > 0 && angle_deg >= 0 && angle_deg <= 360) {
            LidarPoint p;
            p.angle = angle_deg * M_PI / 180.0;
            p.distance = dist_mm / 1000.0;
            scan.push_back(p);
        }

        i += PACKET_SIZE;
    }

    if (i > 0) buffer.erase(buffer.begin(), buffer.begin() + i);

    return !scan.empty();
}
*/
