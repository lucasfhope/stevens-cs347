#include <iostream>
#include <map>
#include <string>

#include "vehicle.cpp"

using namespace std;

class SystemManagement {

    private:
    
    bool loggedIn;
    string user;
    
    string username;
    string password;
    
    map<string,string> passwords;
    map<string,string> users;
    
    void init_credentials()
    {
        passwords["lhope"] = "171717";
        passwords["falcon"] = "goblin";
        passwords["jlee"] = "who?";
        passwords["danny"] = "dimez";
        
        users["lhope"] = "Lucas Hope";
        users["falcon"] = "Steven DeFalco";
        users["jlee"] = "Jude Lee";
        users["danny"] = "Daniel Storms";
    }

    public:
    
    SystemManagement() {
        loggedIn = false;
        init_credentials();
    }
    
    void login_prompt()
    {
        while(!loggedIn)
        {
            cout << "\033[2J\033[1;1H";
            cout << "Vehicle System Management" << endl;
            cout << "Username: ";
            cin >> username;
            cout << "Password: ";
            cin >> password;
            
            if(passwords.count(username) == 0) {
                cout << "Login Failed: Invalid username." << endl;
            } else if(passwords[username].compare(password) == 0) {
                loggedIn = true;
                user = users[username];
            } else {
                cout << "Login Failed: Invalid password." << endl;
            }
        }
    }
    string get_user() { return user; }
};

int main(int argc, char *argv[]) {
    
    int total_steps = 100;
    int curr_step = 0;
    SystemManagement system;
    system.login_prompt();
    
    cout << "\nWelcome " << system.get_user() << endl;
    while (curr_step <= total_steps) {
        float percent_complete = static_cast<float>(curr_step) / total_steps * 100;
        std::cout << "[";
        int bar_width = 30;
        int num_symbols = static_cast<int>(percent_complete / 100 * bar_width);
        for (int i = 0; i < bar_width; ++i) {
            if (i < num_symbols) {
                std::cout << "=";
            } else {
                std::cout << " ";
            }
        }
        std::cout << "] " << percent_complete << "%\r";
        if (percent_complete == 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        curr_step++;
        std::cout.flush();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    Planning running_vehicle = Planning();

    running_vehicle.run_systems();
}
