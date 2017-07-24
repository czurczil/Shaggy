#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <stack>
#include <list>
#include <vector>
#include <string>
#include <cstdlib>

using namespace std;

typedef struct _loop_param
{
    int ptr;
    int count;
} LoopParam;
class prog_parser
{
public:
    map<string,float> variables;
    map<string,int> procedure_ptrs;
    stack<int> program_stack;
    stack<LoopParam> loop_stack;
    vector<string> program_lines;
    int curr_ptr, curr_loop_ptr, curr_loop_counter;


    void clear_all()
    {
        variables.clear();
        procedure_ptrs.clear();
        while (!program_stack.empty()) program_stack.pop();
        while(!loop_stack.empty()) loop_stack.pop();
        program_lines.clear();
        program.clear();
        curr_ptr=0;
        curr_loop_ptr=0;
        curr_loop_counter=0;
    }
    string trim(string& str)
    {
        if(str.size() == 0) return "";
        size_t first = str.find_first_not_of(' ');
        size_t last = str.find_last_not_of(' ');
        if(first >=0 && last >= first)
            return str.substr(first, (last-first+1));
        return "";
    }
    void parse_variable(string line)
    {
        line = line.erase(0,1); //remove $
        int pos=line.find('=');
        string name = line.substr(0,pos);
        string val = line.substr(pos+1);
        float fval=(float)atof(val.c_str());
        variables[name]=fval;
    }
    void create_variables()
    {
        bool var_section=false;
        for(int i=0; i<(int)program_lines.size(); i++)
        {
            string line = program_lines[i];
            if(line == "VARIABLES")
            {
                var_section=true;
                continue;
            }
            if(var_section)
            {
                if(line == "VARIABLES_END") break;
                parse_variable(line);
            }

        }
        cout<<"VARIABLES:"<<endl;
        std::map<std::string, float>::iterator iter;

        for (iter = variables.begin(); iter != variables.end(); ++iter)
        {
            std::cout << iter->first << " => " << iter->second << endl;
        }
    }

    void create_procedure_pointers()
    {
        for(int i=0; i<(int)program_lines.size(); i++)
        {
            string line = program_lines[i];
            if(line.find("proc") == 0)
            {
                int pos = line.find_first_of(' ');
                if(pos > 0)
                {
                    string name = line.substr(pos+1);
                    procedure_ptrs[name]=i+1;
                }
            }
        }
        cout<<"PROCEDURE POINTERS:"<<endl;
        std::map<std::string, int>::iterator iter;

        for (iter = procedure_ptrs.begin(); iter != procedure_ptrs.end(); ++iter)
        {
            std::cout << iter->first << " => " << iter->second << endl;
        }
    }

    int read_file(string filename)
    {
        program_lines.clear();
        fstream plik( filename.c_str(), std::ios::in );
        if( plik.good() == true )
        {
            string linia;
            while( !plik.eof() )
            {
                getline( plik, linia );
                linia=trim(linia);
                if(!linia.empty())
                    program_lines.push_back(linia);
            }
            plik.close();
        }
        else
        {
            cout<<"nie znaleziono"<<endl;
        }
//	for(int i=0;i<(int)program_lines.size();i++)
//	{
//		cout<<program_lines[i]<<endl;
//	}
        return 0;
    }

    enum COMMAND_TYPE {CMD_UNKNOWN,CMD_VARIABLE,CMD_SET,CMD_CORRECTION,CMD_TURN,CMD_PROCEDURE, CMD_PROCEDURE_END, CMD_LOOP, CMD_LOOP_END};

    COMMAND_TYPE detect_command_type(string el)
    {
        COMMAND_TYPE res = CMD_UNKNOWN;
        if(el[0]=='$') res = CMD_VARIABLE;
        else if(el.compare("set") == 0) res = CMD_SET;
        else if(el.compare("cor") == 0) res = CMD_CORRECTION;
        else if(el.compare("turn") == 0) res = CMD_TURN;
        else if(el.compare("call") == 0) res = CMD_PROCEDURE;
        else if(el.compare("end_proc") == 0) res = CMD_PROCEDURE_END;
        else if(el.compare("loop") == 0) res = CMD_LOOP;
        else if(el.compare("end_loop") == 0) res = CMD_LOOP_END;

        return res;
    }

    void print_vect(vector<string> v)
    {
        cout<<"Vector: ";
        for(int i=0; i<(int)v.size(); i++)
        {
            cout<<v[i]<<" ";
        }
        cout<<endl;
    }
    vector<string> parse_line(string line)
    {
        vector<string> res;
        int pos_start=0;
        int pos_end=0;
        do
        {
            pos_end=line.find(' ', pos_start);
            if(pos_end > pos_start)
            {
                res.push_back(line.substr(pos_start,pos_end-pos_start));
                pos_start=pos_end+1;
            }
        }
        while(pos_end > 0);
        res.push_back(line.substr(pos_start));//last element
        return res;

    }
    int get_prog_start_ptr()
    {
        int res = 0;

        for(int i=0; i<(int)program_lines.size(); i++)
        {
            string line = program_lines[i];
            if(line == "PROGRAM")
            {
                res=i+1;
                break;
            }
        }
        return res;
    }
    int get_prog_end_ptr()
    {
        int res = 0;

        for(int i=0; i<(int)program_lines.size(); i++)
        {
            string line = program_lines[i];
            if(line == "PROGRAM_END")
            {
                res=i;
                break;
            }
        }
        return res;
    }

    vector<string> program;

    vector<string> get_program()
    {
        return program;
    }
string prepare_line(vector<string> data)
{
    string res = data[0];
    for(int i=1;i<(int)data.size();i++)
    {
        string val = data[i];
        if(val[0]=='$')
        {
            std::ostringstream ss;
            ss << variables[val.substr(1)];
            std::string s(ss.str());
             res = res.append(" "+s);
        }
        else
        {
            res = res.append(" "+val);
        }
    }

    return res;
}
    vector<string> parse(string filename)
    {
        clear_all();
        read_file(filename);
        create_variables();
        create_procedure_pointers();

         program.reserve(500);

        int prog_start_ptr  = get_prog_start_ptr();
        int prog_end_ptr    = get_prog_end_ptr();
        curr_ptr=prog_start_ptr;
        while(curr_ptr != prog_end_ptr)
        {
            string line = program_lines[curr_ptr];
            cout<<line<<endl;
            vector<string> data = parse_line(line);
            if(data.size() == 0) continue;
            COMMAND_TYPE cmd_type = detect_command_type(data[0]);
            switch(cmd_type)
            {
            case CMD_VARIABLE:
                parse_variable(data[0]);
                curr_ptr++;
                break;
            case CMD_SET:
                curr_ptr++;
                program.push_back(prepare_line(data));
                break;
            case CMD_CORRECTION:
                curr_ptr++;
                program.push_back(prepare_line(data));
                break;
            case CMD_TURN:
                curr_ptr++;
                program.push_back(prepare_line(data));
                break;
            case CMD_PROCEDURE:
                program_stack.push(curr_ptr+1); // push next command to stack
                curr_ptr=procedure_ptrs[data[1]];
                break;
            case CMD_PROCEDURE_END:
                curr_ptr = program_stack.top(); // pop ptr from stack after procedure
                program_stack.pop();
                break;
            case CMD_LOOP:
                LoopParam lp_start;
                lp_start.ptr=curr_loop_ptr;
                lp_start.count=curr_loop_counter;
                loop_stack.push(lp_start);
                curr_loop_ptr=curr_ptr+1;
                curr_loop_counter=atoi(data[1].c_str());
                curr_ptr++;
                break;
            case CMD_LOOP_END:
                if(curr_loop_counter>1)
                {
                    curr_loop_counter--;
                    curr_ptr=curr_loop_ptr;
                }
                else
                {
                    LoopParam lp_end = loop_stack.top();
                    loop_stack.pop();
                    curr_loop_counter=lp_end.count;
                    curr_loop_ptr=lp_end.ptr;
                    curr_ptr++;
                }
                break;
            default:
                cout<<"Unknown command"<<endl;
                curr_ptr++;

                break;
            }

            //print_vect(data);
        }

        return program;
    }

};
/*
int main(void)
{


// PROGRAM
    cout<<"PROGRAM:"<<endl;
    prog_parser parser;
    vector<string> program = parser.parse("walc.txt");
    for(int i=0;i<(int)program.size();i++)
    {
    cout<<program[i]<<endl;
    }


    cout<<"END PROGRAM"<<endl;








    return 0;
}
*/
