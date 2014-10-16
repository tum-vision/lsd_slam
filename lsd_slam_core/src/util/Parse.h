/*
 * Parse.h
 *
 *  Created on: 14 Aug 2014
 *      Author: thomas
 */

#ifndef PARSE_H_
#define PARSE_H_

#include <dirent.h>
#include <string>
#include <boost/filesystem.hpp>

class Parse
{
    public:
        static int arg(int argc, char** argv, const char* str, std::string &val)
        {
            int index = findArg(argc, argv, str) + 1;

            if(index > 0 && index < argc)
            {
                val = argv[index];
            }

            return index - 1;
        }

    private:
        Parse() {}

        static int findArg(int argc, char** argv, const char* argument_name)
        {
            for(int i = 1; i < argc; ++i)
            {
                // Search for the string
                if(strcmp(argv[i], argument_name) == 0)
                {
                    return i;
                }
            }
            return -1;
        }
};

#endif /* PARSE_H_ */
