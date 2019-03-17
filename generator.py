import argparse
import json
import copy

class_vertex = 'Vertex'
vector_vertexes = 'vertexes_'
vector_edges = 'edges_'
emplace_back = 'emplace_back'
map_name = 'vertexes_by_name_'


def generator_file(vertex_file_name, edge_file_name, out):
    with open('sources/defaultPosition.json') as f:
        default_dict_motors = json.load(f)
    with open('sources/defaultOrder.json') as f:
        default_list_order = json.load(f)
    
    vertex_numbers = dict()
    with open(vertex_file_name) as file:
        cur_vertex_name = None
        cur_vertex_dict = copy.deepcopy(default_dict_motors)
        number = 0
        for line in file:
            line_copy = line
            if line == '\n' or line.strip()[0] == '/' and line.strip()[1] == '/':
                continue
            line = line.split()
            if cur_vertex_name is None:
                if len(line) != 2 or line[1] != '{':
                    print('Incorrect line: ' + line_copy + '\n' + 'Should be: \'NAME {\'')
                    raise KeyError
                cur_vertex_name = line[0]
                vertex_numbers[cur_vertex_name] = number
                out.write(map_name + '["' + cur_vertex_name + '"] = ' + str(number) + ';\n')
                number = number + 1
            elif line[0] != '}':
                if line[0] not in default_list_order:
                    print('No such motor: ' + line[0] + '. In vertex ' + cur_vertex_name + '.')
                    raise KeyError
                cur_vertex_dict[line[0]] = line[2]
            else:
                s = ''
                for name in default_list_order:
                    s = s + str(cur_vertex_dict[name]) + ', '
                s = s[:-2]
                out.write(vector_vertexes + '.' + emplace_back + '(std::vector<float>({' + s + '}));\n')
                # МИША) ЗАГЛЯНИ СЮДА
                out.write(vector_vertexes + '.back().SetName("' + cur_vertex_name + '");\n')
                cur_vertex_name = None
                cur_vertex_dict = copy.deepcopy(default_dict_motors)

    out.write('\n')
    with open(edge_file_name) as file:
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


def start_generator_dir(vertex_dir_name, edge_dir_name, output_dir):
    out = open(output_dir, "w")
    generator_file(vertex_dir_name, edge_dir_name, out)
    out.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--vertex', action='store', type=str, help='file with vertexes', required=True)
    parser.add_argument('--edges', action='store', type=str, help='file with edges', required=True)
    parser.add_argument('--out', action='store', type=str, help='first word')
    args = parser.parse_args()
    start_generator_dir(args.vertex, args.edges, args.out)
