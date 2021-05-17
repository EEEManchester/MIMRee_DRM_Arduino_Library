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
} lhm_datastore_t;

lhm_datastore_t dataline;

void printDatastore(basic_ostream<char> &ios)
{
    ios << dataline.timestamp << ",";
    ios << dataline.servo_roll_current << ",";
    ios << dataline.servo_roll_position << ",";
    ios << dataline.servo_roll_velocity << ",";
    ios << dataline.servo_pitch_current << ",";
    ios << dataline.servo_pitch_position << ",";
    ios << dataline.servo_pitch_velocity << ",";
    ios << (unsigned)dataline.hingeStatus << ",";
    ios << (unsigned)dataline.hookStatus << ",";
    ios << (unsigned)dataline.engaged << endl;
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

void decode(string dataFileName)
{
    uint32_t lineCount = 0;
    fstream dataFH;
    ofstream decodeFH;
    dataFH.open(dataFileName, std::fstream::in | std::fstream::binary);
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

    decode("0_0_17.BIN");
    return 0;
}