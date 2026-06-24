#include "Network.h"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

namespace mocap {

UdpReceiver::UdpReceiver(){}
UdpReceiver::~UdpReceiver(){ stop(); }

bool UdpReceiver::start(int port){
    int sock=socket(AF_INET,SOCK_DGRAM,0);
    if(sock<0){std::cerr<<"UDP socket failed\n";return false;}
    int opt=1; setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof(opt));
    sockaddr_in addr{}; addr.sin_family=AF_INET;
    addr.sin_port=htons((uint16_t)port); addr.sin_addr.s_addr=INADDR_ANY;
    if(bind(sock,(sockaddr*)&addr,sizeof(addr))<0){
        std::cerr<<"UDP bind failed on port "<<port<<"\n";
        close(sock); return false;
    }
    running_=true;
    thread_=std::thread([this,sock](){
        uint8_t buf[17];
        sockaddr_in from; socklen_t fl=sizeof(from);
        while(running_){
            ssize_t n=recvfrom(sock,buf,sizeof(buf),0,(sockaddr*)&from,&fl);
            if(n==17){
                int id=buf[0];
                if(id>=0&&id<10){
                    float w,x,y,z;
                    memcpy(&w,buf+1,4);memcpy(&x,buf+5,4);
                    memcpy(&y,buf+9,4);memcpy(&z,buf+13,4);
                    std::lock_guard<std::mutex> lk(mutex_);
                    samples_[id]={w,x,y,z,true};
                }
            }
        }
        close(sock);
    });
    std::cout<<"UDP listening on port "<<port<<"\n";
    return true;
}

void UdpReceiver::stop(){
    running_=false;
    if(thread_.joinable()) thread_.join();
}

std::array<SensorSample,10> UdpReceiver::getAll() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return samples_;
}

} // namespace mocap
