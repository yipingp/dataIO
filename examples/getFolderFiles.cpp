#include "dataIO.hpp"

int main()
{
    dataIO files;
    files.path=".";
    files.getAllFiles(".txt");

    // filename
    for (auto ownname:files.ownnames)
        cout << ownname << endl;

    // directory + filename
    for (auto filepath : files.filepaths)
        cout << filepath << endl;
}