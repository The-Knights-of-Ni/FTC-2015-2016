#include "../jni/misc.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define out_file_suffix "_robot_state_elements.h"
#define java_out_file_prefix "./java/com/qualcomm/ftcrobotcontroller/opmodes/"
#define java_out_file_suffix "RobotStateElements.java"

//char not included because it has different sizes in c and java

enum types{ type_bool, type_byte, type_int, type_long, type_float, type_double, n_types };
const char * type_java_names[] =
{            "boolean",   "byte",     "int",    "long",    "float",    "double"};
const char * type_names[] =
{               "bool",   "byte",     "int",    "long",    "float",    "double"};
const int type_sizes[] =
{                   1,         1,        4,         8,          4,          8};

#define TAB 9 //tab character

#define constStrncmp(a, constant_string, max_len) (strncmp((a), constant_string, min(max_len, sizeof(constant_string)-1)))

#define max_elements 1000
struct element
{
    uint type_id; // 0 < type_id < n_types represent primitive types, type_id > n_types represents user_type[type_id-n_types]
    uint array_len;
    char * name;
    uint name_len;
};

#define max_primitives 1000
struct primitive
{
    types type;
    uint array_len;
    char * name;
    uint name_len;
};

#define max_user_types 500
struct user_type
{
    uint first_primitive;
    uint last_primitive;
    char * name;
    uint name_len;
};

bool8 isComment(char * in, uint i, uint input_size)
{
    return (constStrncmp(in+i, "//", input_size-i) == 0);
}


bool8 isNumber(char c)
{
    return('0' <= c && c <= '9');
}

bool8 isWordStart(char c)
{
    return(c == '_' ||
           ('a' <= c && c <= 'z') ||
           ('A' <= c && c <= 'Z'));
}

bool8 isWhitespace(char c)
{
    return (c == ' ' || c == TAB || c == '\n' || c == '\r');
}

uint nextWordLen(char * s, uint i, uint input_size)
{
    int l = 0;
    for(; i+l < input_size && (isWordStart(s[i+l]) || isNumber(s[i+l])); l++);
    return l;
}

//TODO: optimize hash function
uint32 hash(char * key, uint32 key_size) //currently djb2
{
    uint32 index = 5381;
    
    for(int i = 0; i < key_size; i++)
    {
        index += ((index << 5) + index) ^ key[i];
    }

    return index;
}

void skipOverWhitespaceAndComment(char * in, uint & i, uint input_size)
{
    for(; i < input_size; i++)
    {
        if(isWhitespace(in[i])) continue; //skip over whitespace
        if(isComment(in, i, input_size))
        {
            while(i < input_size && in[i++] != '\n');
            continue;
        }
        break;
    }
}

//NOTE: does not check if string is really a number
int stringToInt(char * s, uint len)
{
    int out = 0;
    int current_tens = 1;
    for(int c = len-1; c >= 0; c--)
    {
        out += (s[c]-'0')*current_tens;
        current_tens *= 10;
    }
    return out;
}

void evalLine(char * in, uint & i, uint input_file_size,
             uint word_len, user_type * user_type_table, 
             primitive * primitives, uint & n_primitives,
             element * elements, uint & n_elements)
{    
    for(int t = 0; t < n_types; t++)
    {
        if(word_len == strlen(type_names[t]) && strncmp(in+i, type_names[t], word_len) == 0)
        { //TODO: error when not found
            i += word_len;
            elements[n_elements].type_id = t;
        
            skipOverWhitespaceAndComment(in, i, input_file_size);
            if(in[i] == '[') //array type
            {
                i++;
                char * array_size_string = in+i;
                uint array_size_string_len = i;
                for(; i < input_file_size && in[i] != ']'; i++)
                {
                    if(!isNumber(in[i]))
                    {
                        printf("error, expected number\n");
                        exit(EXIT_SUCCESS);
                    }
                }
                array_size_string_len = i-array_size_string_len;
                i++;
                elements[n_elements].array_len = stringToInt(array_size_string, array_size_string_len);
                skipOverWhitespaceAndComment(in, i, input_file_size);
            }
            else
            {
                elements[n_elements].array_len = 1;
            }
            if(isWordStart(in[i]))
            {
                elements[n_elements].name = in+i;
                uint elment_name_len = nextWordLen(in, i, input_file_size);
                elements[n_elements].name_len = elment_name_len;
                i += elment_name_len;
            }
            else
            {
                printf("error, expected element name\n");
                exit(EXIT_SUCCESS);
            }
            skipOverWhitespaceAndComment(in, i, input_file_size);
            if(in[i] != ';')
            {
                printf("error, expected ';'\n");
                exit(EXIT_SUCCESS);
            }
            n_elements++;
            return;
        }
    }
    
    uint current_user_type_index = hash(in+i, word_len)%max_user_types;
    uint hash_index = hash(in+i, word_len)%max_user_types;
    for(int ut = hash_index; ut != hash_index-1; (++ut)%=max_user_types)
    {
        
        if(user_type_table[ut].name == 0) //no existing type found
        {
            user_type_table[ut].name = in+i;
            user_type_table[ut].name_len = word_len;
            user_type_table[ut].first_primitive = n_primitives;

            i += word_len;
            skipOverWhitespaceAndComment(in, i, 0);
            if(in[i++] != '{')
            {
                printf("error, expected '{'\n");
                exit(EXIT_SUCCESS);
            }
            for(; i < input_file_size; i++)
            {
                if(isWhitespace(in[i])) continue; //skip over whitespace
                if(isComment(in, i, input_file_size))
                {
                    while(i < input_file_size && in[i++] != '\n');
                    continue;
                }
                uint type_name_len = nextWordLen(in, i, input_file_size);
                for(int t = 0; t < n_types; t++)
                {
                    if(type_name_len == strlen(type_names[t]) && strncmp(in+i, type_names[t], type_name_len) == 0)
                    {
                        primitives[n_primitives].type = (types) t; //TODO: give error when not found
                        i += type_name_len;
                        break;
                    }
                }
                skipOverWhitespaceAndComment(in, i, input_file_size);
                if(in[i] == '[') //array type
                {
                    i++;
                    char * array_size_string = in+i;
                    uint array_size_string_len = i;
                    while(i < input_file_size && in[++i] != ']')
                    {
                        if(!isNumber(in[i]))
                        {
                            printf("error, expected number\n");
                            exit(EXIT_SUCCESS);
                        }
                    }
                    array_size_string_len = i-array_size_string_len-1;
                    primitives[n_primitives].array_len = stringToInt(array_size_string, array_size_string_len);
                    skipOverWhitespaceAndComment(in, i, input_file_size);
                }
                else
                {
                    primitives[n_primitives].array_len = 1;
                }                
                if(isWordStart(in[i]))
                {
                    primitives[n_primitives].name = in+i;
                    uint subelement_name_len = nextWordLen(in, i, input_file_size);
                    primitives[n_primitives].name_len = subelement_name_len;
                    i += subelement_name_len;
                    n_primitives++;
                }
                else
                {
                    printf("error, expected subelement name\n"); //TODO: line numbers
                    exit(EXIT_SUCCESS);
                }
                skipOverWhitespaceAndComment(in, i, input_file_size);
                if(in[i] == '}')
                {
                    i++;
                    skipOverWhitespaceAndComment(in, i, input_file_size);
                    
                    if(in[i] != ';')
                    {
                        printf("error, expected ';'\n"); //TODO: line numbers
                        exit(EXIT_SUCCESS);
                    }
                    
                    user_type_table[ut].last_primitive = n_primitives;
                    return;
                }
                if(in[i++] != ',')
                {
                    printf("error, expected ','\n"); //TODO: line numbers
                    exit(EXIT_SUCCESS);
                }
            }
        }
        
        //existing user type found
        if(word_len == user_type_table[ut].name_len && strncmp(user_type_table[ut].name, in+i, word_len) == 0)
        {
            i += word_len;
            elements[n_elements].type_id = ut+n_types;
            skipOverWhitespaceAndComment(in, i, input_file_size);
            if(in[i] == '[') //array type
            {
                i++;
                char * array_size_string = in+i;
                uint array_size_string_len = i;
                while(i < input_file_size && in[++i] != ']')
                {
                    if(!isNumber(in[i]))
                    {
                        printf("error, expected number\n");
                        exit(EXIT_SUCCESS);
                    }
                }
                array_size_string_len = i-array_size_string_len-1;
                elements[n_elements].array_len = stringToInt(array_size_string, array_size_string_len);
                skipOverWhitespaceAndComment(in, i, input_file_size);
            }
            else
            {
                elements[n_elements].array_len = 1;
            }
            if(isWordStart(in[i]))
            {
                elements[n_elements].name = in+i;
                uint elment_name_len = nextWordLen(in, i, input_file_size);
                elements[n_elements].name_len = elment_name_len;
                i += elment_name_len;
            }
            else
            {
                printf("error, expected element name\n");
                exit(EXIT_SUCCESS);
            }
            skipOverWhitespaceAndComment(in, i, input_file_size);
            if(in[i] != ';')
            {
                printf("error, expected ';'\n");
                exit(EXIT_SUCCESS);
            }
            n_elements++;
            return;
        }
    }
    
    printf("error, hash table full\n");
    exit(EXIT_SUCCESS);
}



void evalFile(char * in, uint input_file_size,
              primitive * primitives, uint & n_primitives,
              user_type * user_type_table,
              element * elements, uint & n_elements)
{    
    for(uint i = 0; i < input_file_size; i++)
    {
        if(constStrncmp(in+i, "/*", input_file_size-i)  == 0)
        {
            for(; i < input_file_size; i++)
            {
                if(constStrncmp(in+i, "*/", input_file_size-i) == 0) break;
                if(constStrncmp(in+i, "robot_state_elements", input_file_size-i) == 0)
                {

                    i += sizeof("robot_state_elements")-1;
                    skipOverWhitespaceAndComment(in, i, input_file_size);
                    if(in[i] != '{')
                    {
                        printf("error, expected '{', found %d\n", in[i]); //TODO: line numbers
                        exit(EXIT_SUCCESS);
                    }
                    for(; i < input_file_size; i++)
                    {
                        if(isWhitespace(in[i])) continue; //skip over whitespace
                        if(isComment(in, i, input_file_size))
                        {
                            while(i < input_file_size && in[i++] != '\n');
                            continue;
                        }
                        if(in[i] == '}') return;
                        if(isWordStart(in[i]))
                        {
                            uint word_len = nextWordLen(in, i, input_file_size);
                            evalLine(in, i, input_file_size,
                                     word_len,
                                     user_type_table, 
                                     primitives, n_primitives,
                                     elements, n_elements);
                        }
                    }
                }
            }
        }
    }
}

int main(int n_args, char ** args)
{
    char * input_filename = 0;
    char * output_filename = 0;
    char * java_output_filename = 0;
    
    void * free_memory = malloc(100*1024);
    
    for(int a = 1; a < n_args; a++) //the first argument is the command
    {
        if(args[a][0] == '-')
        {
            switch(args[a][1])
            {
                case 'o':
                    output_filename = args[++a];
            }
        }
        else
        {
            input_filename = args[a];
        }
    }
    if(input_filename == 0)
    {
        printf("error, no input file\n");
        exit(EXIT_SUCCESS);
    }
    
    uint input_path_part_len = 0;
    for(int l = 0; input_filename[l]; l++)
    {
        if(input_filename[l] == '/') input_path_part_len = l+1;
    }

    uint input_name_part_len = 0;
    for(int l = 0; input_filename[l]; l++)
    {
        if(input_filename[l] == '.') input_name_part_len = l;
    }
    
    FILE * output_file;
    {
        if(output_filename == 0)
        {
            output_filename = (char *) free_memory;
            strncpy(output_filename, input_filename, input_name_part_len);
            strncpy(output_filename+input_name_part_len, out_file_suffix, sizeof(out_file_suffix));
        }
        
        output_file = fopen(output_filename, "w+");
        if(!output_file)
        {
            printf("error, could not open file \"%s\"\n", output_filename);
            exit(EXIT_SUCCESS);
        }
    }

    FILE * java_output_file;
    {
        if(java_output_filename == 0)
        {
            java_output_filename = (char *) free_memory;
            strncpy(java_output_filename, java_out_file_prefix, sizeof(java_out_file_prefix)-1);
            strncpy(java_output_filename+sizeof(java_out_file_prefix)-1, input_filename+input_path_part_len, input_name_part_len-input_path_part_len);
            strncpy(java_output_filename+sizeof(java_out_file_prefix)-1+input_name_part_len-input_path_part_len, java_out_file_suffix, sizeof(java_out_file_suffix));
        }
        
        java_output_file = fopen(java_output_filename, "w+");
        if(!java_output_file)
        {
            printf("error, could not open file \"%s\"\n", java_output_filename);
            exit(EXIT_SUCCESS);
        }
    }
    
    FILE * input_file = fopen(input_filename, "r");
    if(!input_file)
    {
        printf("error, could not open file \"%s\"\n", input_filename);
        exit(EXIT_SUCCESS);
    }

    int error = fseek(input_file, 0, SEEK_END);
    assert(error == 0);

    size_t input_file_size = ftell(input_file);
    assert(input_file_size != -1);

    error = fseek(input_file, 0, SEEK_SET);
    assert(error == 0);
    
    char * in = (char *) malloc(input_file_size+1);

    fread(in, sizeof(char), input_file_size, input_file);
    in[input_file_size] = 0;
    
    ////////////////////////////////////////////////////
    #define stalloc(size) free_memory; free_memory = (void*)((char*) free_memory + size);
    
    primitive * primitives = (primitive *) stalloc(sizeof(types)*max_primitives);
    uint n_primitives = 0;

    user_type * user_type_table = (user_type *) stalloc(sizeof(user_type)*max_user_types);
    memset(user_type_table, 0, sizeof(user_type)*max_user_types);

    element * elements = (element *) stalloc(sizeof(types)*max_elements);
    uint n_elements = 0;
    
    evalFile(in, input_file_size,
             primitives, n_primitives,
             user_type_table,
             elements, n_elements);

    printf("elements:\n");
    for(int e = 0; e < n_elements; e++)
    {
        printf("element %d\n", e);
        printf("type_id: %d\n", elements[e].type_id);
        printf("type: %.*s\n", (elements[e].type_id < n_types) ? strlen(type_names[elements[e].type_id]) : user_type_table[elements[e].type_id-n_types].name_len, (elements[e].type_id < n_types) ? type_names[elements[e].type_id] : user_type_table[elements[e].type_id-n_types].name);
        printf("array_len: %d\n", elements[e].array_len);
        elements[e].name[elements[e].name_len] = 0;
        printf("name: %s\n\n", elements[e].name);
    }

    uint robot_state_size = 0;
    
    fprintf(output_file, "\
/*\n\
WARNING: this is a generated file\n\
changes made this file are not permanent\n\
*/\n\
enum robot_state_element\n\
{\n");//TODO: include guards
    for(int e = 0; e < n_elements; e++)
    {
        if(elements[e].type_id < n_types)
        {
            fprintf(output_file, "\
    rsid_%.*s, rsid_%.*s_end = rsid_%.*s + %d,\n",
                    elements[e].name_len, elements[e].name,
                    elements[e].name_len, elements[e].name,
                    elements[e].name_len, elements[e].name,
                    type_sizes[elements[e].type_id]*elements[e].array_len);
            robot_state_size += type_sizes[elements[e].type_id]*elements[e].array_len;
        }
        else
        {
            uint user_type_size = 0;
            for(int p = user_type_table[elements[e].type_id-n_types].first_primitive; p < user_type_table[elements[e].type_id-n_types].last_primitive; p++)
            {
                user_type_size += type_sizes[primitives[p].type]*primitives[p].array_len;
            }
            robot_state_size += user_type_size;
            fprintf(output_file, "\
    rsid_%.*s, rsid_%.*s_end = rsid_%.*s + %d,\n",
                    elements[e].name_len, elements[e].name,
                    elements[e].name_len, elements[e].name,
                    elements[e].name_len, elements[e].name,
                    user_type_size*elements[e].array_len);
        }
    }
    fprintf(output_file, "\
    rsid_size\n\
};\n\
\n\
#include \"jni_functions.h\"\n\
\n\
\n");    

    for(int e = 0; e < n_elements; e++)
    {
        if(elements[e].type_id < n_types)
        {
            fprintf(output_file, "\
%s & %.*s = ((%s *) robot_state.state)[rsid_%.*s];\n",
                    type_names[elements[e].type_id],
                    elements[e].name_len, elements[e].name,
                    type_names[elements[e].type_id],
                    elements[e].name_len, elements[e].name);
        }
        else
        {
            fprintf(output_file, "\
%.*s & %.*s = ((%.*s *) robot_state.state)[rsid_%.*s];\n",
                    user_type_table[elements[e].type_id-n_types].name_len, user_type_table[elements[e].type_id-n_types].name,
                    elements[e].name_len, elements[e].name,
                    user_type_table[elements[e].type_id-n_types].name_len, user_type_table[elements[e].type_id-n_types].name,
                    elements[e].name_len, elements[e].name);
        }
    }

    //TODO: generate java file
    
    fprintf(java_output_file, "\
/*\n\
WARNING: this is a generated file\n\
changes made this file are not permanent\n\
*/\n\
package com.qualcomm.ftcrobotcontroller.opmodes;\n\
\n\
import java.nio.ByteBuffer;\n\
import java.nio.ByteOrder;\n\
\n\
public class %.*sRobotStateElements\n\
{\n\
    public static byte[] robot_state;\n\
    public static int robot_state_size = %d;\n\
    \n\
    %.*sRobotStateElements(){}\n",
            input_name_part_len-input_path_part_len, input_filename+input_path_part_len,
            robot_state_size,
            input_name_part_len-input_path_part_len, input_filename+input_path_part_len);
    //TODO: support non-default java output filenames
    
    uint current_index = 0;
    for(int e = 0; e < n_elements; e++)
    {
        if(elements[e].array_len == 1)
        {
            if(elements[e].type_id < n_types)
            {
                uint type_size = type_sizes[elements[e].type_id];
            
                fprintf(java_output_file, "\
    public static void set_%.*s(%s value)\n\
    {\n\
        ByteBuffer.wrap(robot_state, %d, %d).order(ByteOrder.nativeOrder()).put%c%s(value);\n\
    }\n\n",
                        elements[e].name_len, elements[e].name,
                        type_names[elements[e].type_id],
                        current_index,
                        type_size,
                        type_names[elements[e].type_id][0]+(
                            type_names[elements[e].type_id][0] >= 'A' ? 'A'-'a' : 0), type_names[elements[e].type_id]+1);
            
                fprintf(java_output_file, "\
    public static %s get_%.*s()\n\
    {\n\
        return ByteBuffer.wrap(robot_state, %d, %d).order(ByteOrder.nativeOrder()).get%c%s();\n\
    }\n\n",
                        type_names[elements[e].type_id],
                        elements[e].name_len, elements[e].name,
                        current_index,
                        type_size,
                        type_names[elements[e].type_id][0]+(
                            type_names[elements[e].type_id][0] >= 'A' ? 'A'-'a' : 0), type_names[elements[e].type_id]+1);
            
                current_index += type_size;
            }
            else
            {
                for(int p = user_type_table[elements[e].type_id-n_types].first_primitive; p < user_type_table[elements[e].type_id-n_types].last_primitive; p++)
                {
                    fprintf(java_output_file, "\
    public static void set_%.*s_%.*s(%s value)\n\
    {\n\
        ByteBuffer.wrap(robot_state, %d, %d).order(ByteOrder.nativeOrder()).put%c%s(value);\n\
    }\n\n",
                            elements[e].name_len, elements[e].name,
                            primitives[p].name_len, primitives[p].name,
                            type_names[primitives[p].type],
                            current_index,
                            type_sizes[primitives[p].type],
                            type_names[primitives[p].type][0]+(
                                type_names[primitives[p].type][0] >= 'A' ? 'A'-'a' : 0), type_names[primitives[p].type]+1);
            
                    fprintf(java_output_file, "\
    public static %s get_%.*s_%.*s()\n\
    {\n\
        return ByteBuffer.wrap(robot_state, %d, %d).order(ByteOrder.nativeOrder()).get%c%s();\n\
    }\n\n",
                            type_names[primitives[p].type],
                            elements[e].name_len, elements[e].name,
                            primitives[p].name_len, primitives[p].name,
                            current_index,
                            type_sizes[primitives[p].type],
                            type_names[primitives[p].type][0]+(
                                type_names[primitives[p].type][0] >= 'A' ? 'A'-'a' : 0), type_names[primitives[p].type]+1);
                
                    current_index += type_sizes[primitives[p].type]*primitives[p].array_len;
                }            
            }
        }
        else //array
        {
            if(elements[e].type_id < n_types)
            {
                uint type_size = type_sizes[elements[e].type_id]*elements[e].array_len;

                fprintf(java_output_file, "\
    public static %s[] get_%.*s()\n\
    {\n",
                        type_names[elements[e].type_id],
                        elements[e].name_len, elements[e].name);
                
                if(elements[e].type_id != type_byte)
                {
                    fprintf(java_output_file, "\
        return ByteBuffer.wrap(robot_state, %d, %d).order(ByteOrder.nativeOrder()).as%c%sBuffer().array();\n",
                            current_index,
                            type_size,
                            type_names[elements[e].type_id][0]+(
                                type_names[elements[e].type_id][0] >= 'A' ? 'A'-'a' : 0), type_names[elements[e].type_id]+1);
                }
                else
                {
                    fprintf(java_output_file, "\
        return ByteBuffer.wrap(robot_state, %d, %d).order(ByteOrder.nativeOrder()).array();\n",
                            current_index,
                            type_size);
                }
                fprintf(java_output_file,"\
    }\n\n");
                
                current_index += type_size;
            }
            else
            {
                printf("error, non-primitive arrays are not yet supported\n");
            }
        }
    }
    fprintf(java_output_file, "\
}");
    
    fclose(input_file);
    fclose(output_file);
    fclose(java_output_file);
}
