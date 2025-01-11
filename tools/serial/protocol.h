enum Command {
    // Terminates anything running (program, command, etc.)
    HALT = 0,
    // Uploads a program
    UPLOAD = 1,
    // Sets a key and a value - defined beforehand

};

enum Role {
    SERVER,
    ROBOT
};

enum Status {

};

class Protocol {
private:
    Role role;
    Status status;

public:
    Protocol(Role role) : role(role) {};
};