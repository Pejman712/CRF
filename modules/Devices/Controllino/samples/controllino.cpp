#include <iostream>

#include <Controllino/Controllino.hpp>
#include <IPC/FIFO.hpp>
#include <IPC/MMAP.hpp>

using namespace crf::devices;

std::atomic<bool> quit(false); 

void my_handler(int s){
    quit.store(true);
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cout << "Wrong number of parameters. Usage:\n";
        std::cout << "./controllino <port_name (e.g. /dev/ttyACM0)> <fifo_filename> <mmap_filename>\n";
        return -1;
    }

    std::string dev_name (argv[1]);
    std::shared_ptr<Controllino> contr = std::make_shared<Controllino>(dev_name, FIFO::CreateReaderPtr(argv[2]), MMAP::CreateWriterPtr(argv[3]));
    if (!contr->initialize()) return -1;
    
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    while(!quit.load()) {
        sleep(1);
    }

    contr->deinitialize();
    

    return 0;    
}