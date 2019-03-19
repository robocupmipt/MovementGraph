import argparse
import json
import copy

class_vertex = 'Vertex'
vector_vertexes = 'vertexes_'
vector_edges = 'edges_'
emplace_back = 'emplace_back'
map_name = 'vertexes_by_name_'
make_pair = 'std::make_pair'
stiffness = [1] * 100 # MaxLen


def generator(vertex_file, edge_file, output):
    if output is None:
        output = "automaticInitGeneration.h"
    with open('sources/defaultPosition.json') as f:
        default_dict_motors = json.load(f)
    with open('sources/defaultOrder.json') as f:
        default_list_order = json.load(f)
    out = open(output, 'w', encoding='utf-8')

    out.write('// generated code begins\n\n')

    vertex_numbers = dict()
    with open(vertex_file) as file:
        cur_vertex_name = None
        cur_vertex_dict = copy.deepcopy(default_dict_motors)
        cur_vertex_stiff = dict()
        for name in default_list_order:
            cur_vertex_stiff[name] = 1
        number = 0
        for line in file:
            line_copy = line

            # Comment
            if line == '\n' or line.strip()[0] == '/' and line.strip()[1] == '/':
                continue

            line = line.split()

            # Beginning of vertex struct
            if cur_vertex_name is None:
                if len(line) != 2 or line[1] != '{':
                    print('Incorrect line: ' + line_copy + '\n' + 'Should be: \'NAME {\'')
                    raise KeyError
                cur_vertex_name = line[0]
                vertex_numbers[cur_vertex_name] = number
                out.write(map_name + '["' + cur_vertex_name + '"] = ' + str(number) + ';\n')
                number = number + 1


            # Middle
            elif line[0] != '}':
                if line[0] not in default_list_order:
                    print('No such motor: ' + line[0] + '. In vertex ' + cur_vertex_name + '.')
                    raise KeyError
                cur_vertex_dict[line[0]] = line[2]
                if len(line) > 3:
                    cur_vertex_stiff[line[0]] = line[3]

        
            # End: writing
            else:
                s = ''
                for name in default_list_order:
                    s = s + make_pair + '(' + str(cur_vertex_dict[name]) + ', ' + str(cur_vertex_stiff[name]) + '), '
                s = s[:-2]
                out.write(vector_vertexes + '.' + emplace_back + '(std::vector<std::pair<float, float> >({' + s + '}));\n')
                out.write(vector_vertexes + '.back().SetName("' + cur_vertex_name + '");\n')       
                cur_vertex_name = None
                cur_vertex_dict = copy.deepcopy(default_dict_motors)

    out.write('\n')
    with open(edge_file) as file:
        for line in file:
            line_copy = line
            if line == '\n' or line.strip()[0] == '/' and line.strip()[1] == '/':
                continue
            line = line.split()
            try:
                vertex_from = '&' + vector_vertexes + '[' + str(vertex_numbers[line[0]]) + ']'
                vertex_to = '&' + vector_vertexes + '[' + str(vertex_numbers[line[1]]) + ']'
                out.write(vector_edges + '.' + emplace_back + '(' + vertex_from + ', ' + vertex_to + ', ' + line[2] + ');\n')
            except KeyError as error:
                print('Sorry, there is no such vertex ' + error.args[0] + ', like in  line: ' + line_copy)
                raise error

    out.write('// generated code ends\n\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--vertex', action='store', type=str, help='file with vertexes', required=True)
    parser.add_argument('--edges', action='store', type=str, help='file with edges', required=True)
    parser.add_argument('--out', action='store', type=str, help='first word')
    args = parser.parse_args()
    generator(args.vertex, args.edges, args.out)
