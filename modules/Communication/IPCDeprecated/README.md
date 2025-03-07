# IPC: Inter-Process Communication {#inter_process_comm}

### FIFO Using ZMQ

A class to manage FIFO communication using ZMQ has been developed. The using of this class has the same procedure as the FIFO class previously created.

### MMAP Using ZMQ

A class to manage MMAP communication using ZMQ has been developed. This class has two ways to use it:

* If the writer and receiver are in differents programs, then the procedure to code is the same as the previous MMAP:

```
    // In one program
    std::shared_ptr<IPC> writer;
    writer = crf::communication::ipc::MMAPZMQ::CreateWriterPtr("mmap_testing");
    // In another program:
    std::shared_ptr<IPC> reader;
    reader = crf::communication::ipc::MMAPZMQ::CreateReaderPtr("mmap_testing");

```

* If the writer and reader are in the same program, it has to have in the address "inproc://": 

```
    std::shared_ptr<IPC> writer;
    std::shared_ptr<IPC> reader;
    writer = crf::communication::ipc::MMAPZMQ::CreateWriterPtr("inproc://ProcessTest");
    reader = crf::communication::ipc::MMAPZMQ::CreateReaderPtr("inproc://ProcessTest");
```
