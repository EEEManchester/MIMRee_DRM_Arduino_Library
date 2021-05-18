#include <iostream>
#include <fstream>
using namespace std;

typedef struct Datastore
{
  uint32_t timestamp;
  float servo_roll_current;
  float servo_roll_position;
  float servo_roll_velocity;
  float servo_pitch_current;
  float servo_pitch_position;
  float servo_pitch_velocity;
  uint8_t hingeStatus;
  uint8_t hookStatus;
  bool engaged;
  uint8_t lastCommand;
  uint32_t lastCommandTime;
  uint32_t lastCommandGCSTime;
} lhm_datastore_t;

lhm_datastore_t dataline;

void printDatastore(basic_ostream<char> &ios)
{
    ios << (unsigned)dataline.timestamp << ",";
    ios << dataline.servo_roll_current << ",";
    ios << dataline.servo_roll_position << ",";
    ios << dataline.servo_roll_velocity << ",";
    ios << dataline.servo_pitch_current << ",";
    ios << dataline.servo_pitch_position << ",";
    ios << dataline.servo_pitch_velocity << ",";
    ios << (unsigned)dataline.hingeStatus << ",";
    ios << (unsigned)dataline.hookStatus << ",";
    ios << (unsigned)dataline.engaged << ",";
    ios << (unsigned)dataline.lastCommand << ",";
    ios << (unsigned)dataline.lastCommandTime << ",";
    ios << (unsigned)dataline.lastCommandGCSTime << endl;
}

void createDecodeFile(string dataFileName, ofstream &fh)
{
    string decodedFileName = "decoded_" + dataFileName + ".txt";
    cout << decodedFileName;

    fh.open(decodedFileName);
    if (!fh.is_open())
    {
        cout << "cannot be created" << endl;
        return;
    }
    cout << " created." << endl;
}

void decode(string folder, string dataFileName)
{
    uint32_t lineCount = 0;
    fstream dataFH;
    ofstream decodeFH;
    dataFH.open(folder + "\\" + dataFileName, std::fstream::in | std::fstream::binary);
    if (!dataFH)
    {
        cout << "Error, file does not exist." << endl;
        return;
    }
    createDecodeFile(dataFileName, decodeFH);

    while (dataFH.read((char *)&dataline, sizeof(dataline)))
    {
        printDatastore(decodeFH);
        lineCount++;
        if (lineCount % 1000 == 1)
        {
            cout << "Decoding in progress: Decoded " << lineCount << " entries. Data peek: ";
            printDatastore(cout);
        }
    }
    dataFH.close();
    decodeFH.close();
    cout << "File decoded, " << lineCount << " entries." << endl;
}

int main()
{
    string folder = "F:\\LHM5";
    decode(folder, "0_2_55.BIN");
    return 0;
}