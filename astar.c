/*
 *    INCLUDE EXTERNAL LIBRAIRIES
 */

#include <stdio.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>


/*
 *    DEFINE CONSTANT
 */

#define HEIGHT_arg 1
#define WIDTH_arg 2
#define X_START_arg 4
#define Y_START_arg 3
#define X_TARGET_arg 6
#define Y_TARGET_arg 5
#define SEED_arg 7
#define RAND_OBSTACLES 8



/*
 *    DEFINE MACROS
 */

#define ERR(msg) printf("%s\n",msg); return -1

#define IS_EMPTY(list) (list[0].pos.x == -1)
#define COPY_COORD(pos_a, pos_b) (pos_a.x) = (pos_b.x); (pos_a.y) = (pos_b.y)
#define LIST_END(cell) ((cell.pos.x == -1) && (cell.pos.y == -1))

#define EUCLIDIAN_DISTANCE(p1, p2) (sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)))

/*
 *    DEFINES STRUCTURES/ENUMS
 */

typedef enum dir {
    UP,
    UP_LEFT,
    UP_RIGHT,
    RIGHT,
    LEFT,
    DOWN,
    DOWN_LEFT,
    DOWN_RIGHT
}dir_e;

typedef enum cell {
    EMPTY = 0,
    START,
    TARGET, OBSTACLE,
    PATH
}cell_e;

typedef struct coord {
    int x;
    int y;
}coord_t;

typedef struct map {
    int height;
    int width;

    int nb_obstacles;
    coord_t start;
    coord_t target;

    cell_e **cells;
}map_t;

typedef struct cell_astar {
    coord_t pos;

    float heuristique;
    float cost;

} cell_astar_t;

/*
 *    DECLARATIONS FUNCTIONS
 */

int parse_arg(int argc, char * argv[], map_t *board, coord_t *start, coord_t *target, unsigned int *seed); 
void map_show(map_t board); 
void map_free(map_t board);
int map_init(map_t * board, coord_t start, coord_t target); 

int init_astar_path(const map_t board, cell_astar_t **openlist, cell_astar_t **closelist); 
int add_to_list(cell_astar_t *list, cell_astar_t cell, size_t max_size_list); 
int pop_lowest_heuristic_from_list(cell_astar_t *list, size_t max_size_list, cell_astar_t *popped_cell); 
int remove_from_list(cell_astar_t *list, cell_astar_t cell, size_t max_size_list); 
int on_target(coord_t actual_cell, coord_t target_cell); 
int create_astar_path(map_t *board); 

/*          PROGRAM BEGIN       */

int parse_arg(int argc, char * argv[], map_t *board, coord_t *start, coord_t *target, unsigned int *seed) {
    if (argc < 8) {
        ERR("Please enter in order:\n./prog HEIGHT WIDTH START_X START_Y TARGET_X TARGET_Y SEED NB_OBSTACLES(optional)");
    }
    
    board->height = atoi(argv[HEIGHT_arg]);
    board->width = atoi(argv[WIDTH_arg]);
    start->x = atoi(argv[X_START_arg]);
    start->y = atoi(argv[Y_START_arg]);
    target->x = atoi(argv[X_TARGET_arg]);
    target->y = atoi(argv[Y_TARGET_arg]);
    *seed = atoi(argv[SEED_arg]);


    if (board->height < 0 || board->width < 0 || start->x < 0 || start->y < 0 || target->x < 0 || target->y < 0) {
        ERR("Given args needs to be positive");
    }

    if (start->x >= board->width || start->y >= board->height){
        ERR("Given start point cannot be placed on the board");
    }

    if (target->x >= board->width || target->y >= board->height){
        ERR("Given target point cannot be placed on the board");
    }

    if ((start->x == target->x) && (start->y == target->y)){
        ERR("start and target are the same");
    }

    if (argc == 9) {
        board->nb_obstacles = atoi(argv[RAND_OBSTACLES]);
    } else {
        board->nb_obstacles = 0;
    }

    return 0;
}

void map_show(map_t board) {
    for (int x_it = 0 ; x_it < board.width ; x_it++) {
        for (int y_it = 0 ; y_it < board.height ; y_it++) {
            switch(board.cells[x_it][y_it]) {
                case EMPTY:
                    printf(".");
                    break;
                
                case START:
                    printf("S");
                    break;

                case TARGET:
                    printf("T");
                    break;

                case OBSTACLE:
                    printf("x");
                    break;

                case PATH:
                    printf("R");
                    break;
            }

            if (y_it == board.height-1)
                printf("\n");
        }
    }
}

void map_free(map_t board){
    for (int i = 0 ; i < board.height ; i++)
        free(board.cells[i]);
    free(board.cells);
}

int map_init(map_t * board, coord_t start, coord_t target) {
    board->cells = malloc(sizeof(cell_e*) * board->width);
    int i = 0, j = 0;
    for (i = 0 ; i < board->width ; i++)
        board->cells[i] = malloc(sizeof(cell_e) * board->height);

    for (i = 0 ; i < board->width ; i++) {
        for (j = 0 ; j < board->height ; j++) {
            board->cells[i][j] = EMPTY;
        }
    }
    
    board->cells[start.x][start.y] = START; 
    board->cells[target.x][target.y] = TARGET;

    coord_t start_point = {start.x, start.y};
    coord_t target_point = {target.x, target.y};

    board->start = start_point;
    board->target = target_point;

    for (int i = 0 ; i < board->nb_obstacles ; i++) {
        int x_obs, y_obs;
        do {
            x_obs = rand() % board->width;
            y_obs = rand() % board->height;
        } while(board->cells[x_obs][y_obs] != EMPTY);
        board->cells[x_obs][y_obs] = OBSTACLE;
    }

    return 0;
}

int init_astar_path(const map_t board, cell_astar_t **openlist, cell_astar_t **closelist) {
    int i = 0, j = 0;
    size_t size_list = board.height * board.width;

    *openlist = malloc(sizeof(cell_astar_t) * size_list);
    *closelist = malloc(sizeof(cell_astar_t) * size_list);

    for (int i = 0 ; i < size_list ; i++) {
        (*openlist)[i].cost = FLT_MAX;
        (*openlist)[i].heuristique = FLT_MAX;
        (*openlist)[i].pos.x = -1;
        (*openlist)[i].pos.y = -1;

        (*closelist)[i].cost = FLT_MAX;
        (*closelist)[i].heuristique = FLT_MAX;
        (*closelist)[i].pos.x = -1;
        (*closelist)[i].pos.y = -1;
    }

    return 0;
}


int add_to_list(cell_astar_t *list, cell_astar_t cell, size_t max_size_list) {
    int i = 0;
    while (!LIST_END(list[i])) {
        i++;
        if (i == max_size_list) {
           ERR("no more room in add_to_list");
        }
    }

    memcpy(list + i, &cell, sizeof(cell_astar_t));

    return 0;
}

int pop_lowest_heuristic_from_list(cell_astar_t *list, size_t max_size_list, cell_astar_t *popped_cell) {
    int i = 1;

    if (IS_EMPTY(list)) {
        ERR("Empty list given in pop_lowest_heuristic_from_list");
    }

    memcpy(popped_cell, list, sizeof(cell_astar_t));

    while (!LIST_END(list[i])) {
        if (popped_cell->heuristique > list[i].heuristique) {
            memcpy(popped_cell, list + i, sizeof(cell_astar_t));
        }
        i++;
    }

    if (remove_from_list(list, *popped_cell, max_size_list)) {
        ERR("remove_from_list in pop_lowest_heuristic_from_list failed");
    }

    return 0;
}

int is_in_list(cell_astar_t *list, cell_astar_t cell, size_t max_size_list) {
    int i = 0;
    while ((list[i].pos.x != cell.pos.x) || (list[i].pos.y != cell.pos.y)) {
        if (++i == max_size_list) {
            return -1;
        }
    }
    return i;
}

int remove_from_list(cell_astar_t *list, cell_astar_t cell, size_t max_size_list) {
    int i = 0;
    if ((i = is_in_list(list, cell, max_size_list)) == -1) {
        ERR("cell not found in remove_from_list");
    }

    // Remove the cell from the list by setting its pos to negative coords
    list[i].pos.x = -1; list[i].pos.y = -1;

    if ((i + 1) != max_size_list) {
        //shift the list from one to the right
        memmove(list + i, list + i + 1, (max_size_list - (i+1)));
        // Remove last element after the shift
        list[max_size_list - 1].pos.x = -1;
        list[max_size_list - 1].pos.y = -1;
    }

    return 0;
}

int on_target(coord_t actual_cell, coord_t target_cell) {
    if ((actual_cell.x == target_cell.x) && (actual_cell.y == target_cell.y)) {
        return 1;
    }
    return 0;
}

int get_pos_from_dir(coord_t pos, coord_t *new_pos, dir_e direction, map_t board, int width, int height) {
    switch(direction) {
        case UP:
            if (pos.y == 0) return 0;
            new_pos->x = pos.x;
            new_pos->y = pos.y - 1;
        break;

        case UP_LEFT:
            if (pos.y == 0 || pos.x == 0) return 0;
            new_pos->x = pos.x - 1;
            new_pos->y = pos.y - 1;
        break;

        case UP_RIGHT:
            if (pos.y == 0 || pos.x == width) return 0;
            new_pos->x = pos.x + 1;
            new_pos->y = pos.y - 1;
        break;

        case RIGHT:
            if (pos.x == width) return 0;
            new_pos->x = pos.x + 1;
            new_pos->y = pos.y;
        break;
    
        case LEFT:
            if (pos.x == 0) return 0;
            new_pos->x = pos.x - 1;
            new_pos->y = pos.y;
        break;

        case DOWN:
            if (pos.y == height) return 0;
            new_pos->x = pos.x;
            new_pos->y = pos.y + 1;
        break;

        case DOWN_LEFT:
            if (pos.y == height || pos.x == 0) return 0;
            new_pos->x = pos.x - 1;
            new_pos->y = pos.y + 1;
        break;

        case DOWN_RIGHT:
            if (pos.y == height || pos.x == width) return 0;
            new_pos->x = pos.x + 1;
            new_pos->y = pos.y + 1;
        break;
    }

    if (board.cells[new_pos->x][new_pos->y] == OBSTACLE) {
        return 0;
    }

    return 1; 
}

int add_neighbours(cell_astar_t *openlist, cell_astar_t *closelist, cell_astar_t actual_cell, map_t board) {
    int max_size_list = board.width * board.height;
    int i;
    coord_t neighbour_pos;
    cell_astar_t neighbour;

    for (dir_e direction = UP ; direction <= DOWN_RIGHT ; direction++) {
        if (get_pos_from_dir(actual_cell.pos, &neighbour.pos, direction, board, board.width, board.height)) {
            if (is_in_list(closelist, neighbour, max_size_list) >= 0) {
                continue;
            }

            neighbour.cost = actual_cell.cost + 1;  
            neighbour.heuristique = neighbour.cost + EUCLIDIAN_DISTANCE(neighbour.pos, board.target);
        
            // Si neighbour existe dans openlist avec un coût inférieur, ignorer
            if ((i = is_in_list(openlist, neighbour, max_size_list)) != -1) {
                if (openlist[i].cost < neighbour.cost) {
                    continue;
                }
            }

            add_to_list(openlist, neighbour, max_size_list);
        }
    }
}

int render_path(map_t *board, cell_astar_t *closelist, size_t max_size_list) {
    int i = 0;
    while(!LIST_END(closelist[i])) {
        if (board->cells[closelist[i].pos.x][closelist[i].pos.y] != START)
            board->cells[closelist[i].pos.x][closelist[i].pos.y] = PATH;
        i++;
        if (i == max_size_list) {
            ERR("render_path max size found");
        }
    }

    return 0;
}

int create_astar_path(map_t *board) {
    cell_astar_t *openlist;
    cell_astar_t *closelist;
    int res;
    int max_size_list = board->height * board->width;

    if (init_astar_path(*board, &openlist, &closelist)) {
        ERR("init of astar_path failed");
    }

    cell_astar_t start_point = {board->start, EUCLIDIAN_DISTANCE(board->start, board->target), 0};
    add_to_list(openlist, start_point, max_size_list);

    while(!IS_EMPTY(openlist)) {
        cell_astar_t working_cell;
        if((res = pop_lowest_heuristic_from_list(openlist, max_size_list, &working_cell))) {
            ERR("openlist empty in pop_lowest_heuristic_from_list");
        }

        if (on_target(working_cell.pos, board->target)) {
            printf("SUCESS\n");
            
            render_path(board, closelist, max_size_list);
            map_show(*board);
            // Create path
        }

        add_neighbours(openlist, closelist, working_cell, *board);

        add_to_list(closelist, working_cell, max_size_list);
        render_path(board, closelist, max_size_list);
        printf("\n---------------\n");
        map_show(*board);
    }

    free(openlist); free(closelist);

    return 0;
}



int main(int argc, char* argv[]) {

    map_t board;
    coord_t start;
    coord_t target;
    
    unsigned int seed;

    int res = parse_arg(argc, argv, &board, &start, &target, &seed);
    printf("res: %d\n", res);

    srand(seed);

//    if (parse_arg(argc, argv, &board, &start, &target))
//        ERR("parse_arg");
    
    map_init(&board, start, target);

    map_show(board);

    create_astar_path(&board);

    map_free(board);    

    return 0;
}
/*

int free_astar_path(astar_inf_t *info, int width) {
    int i;
    for (i = 0 ; i < width ; i++) {
        free(info->astar_board[i]);
    }
    free(info->astar_board);
    free(info->openList);
    free(info->closeList);
}

astar_inf_t *init_astar_path(const map_t *board) {
    int i, j;
    astar_inf_t *info;
    info->astar_board = malloc(sizeof(cell_astar_t *) * board->width);
    for (i = 0 ; i < board->width ; i++) {
        info->astar_board[i] = malloc(sizeof(cell_astar_t) * board->height);
    }

    info->openList = malloc(sizeof(coord_t) * board->height * board->width);
    info->closeList = malloc(sizeof(coord_t) * board->height * board->width);

    for (i = 0 ; i < board->width ; i++) {
        for (j = 0 ; j < board->height ; j++) {
            info->openList[i*board->width + j].x = -1;
            info->openList[i*board->width + j].y = -1;

            info->closeList[i*board->width + j].x = -1;
            info->closeList[i*board->width + j].y = -1;

            info->astar_board[i][j].f = FLT_MAX;
            info->astar_board[i][j].h = FLT_MAX;
            info->astar_board[i][j].g = FLT_MAX;
            
            info->astar_board[i][j].parent.x = -1;
            info->astar_board[i][j].parent.y = -1;
            
            info->astar_board[i][j].pos.x = i;
            info->astar_board[i][j].pos.y = j;
        }
    }

    return info;
}*/