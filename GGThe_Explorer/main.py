from tkinter import *
import sys
import os.path
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from search import *
from search import breadth_first_tree_search as bfts, depth_first_tree_search as dfts, \
    depth_first_graph_search as dfgs, breadth_first_graph_search as bfs, uniform_cost_search as ucs
from utils import PriorityQueue
from copy import deepcopy

root = None
city_coord = {}
romania_problem = None
algo = None
start = None
goal = None
counter = -1
city_map = None
frontier = None
front = None
node = None
next_button = None
explored = None


def create_map(root):
    """This function draws out the required map."""
    global city_map, start, goal
    IT_Del_locations = map_IT_Del.locations
    width = 780
    height = 600
    margin = 6
    city_map = Canvas(root, width=width, height=height)
    city_map.pack()

    # Since lines have to be drawn between particular points, we need to list
    # them separately


    #Napitupulu
    make_line(
        city_map,
        IT_Del_locations['Napitupulu'][0],
        height -
        IT_Del_locations['Napitupulu'][1],
        IT_Del_locations['Silo'][0],
        height -
        IT_Del_locations['Silo'][1],
        map_IT_Del.get('Napitupulu', 'Silo'))
    make_line(
        city_map,
        IT_Del_locations['Napitupulu'][0],
        height -
        IT_Del_locations['Napitupulu'][1],
        IT_Del_locations['GD9'][0],
        height -
        IT_Del_locations['GD9'][1],
        map_IT_Del.get('Napitupulu', 'GD9'))    

    #Silo
    make_line(
        city_map,
        IT_Del_locations['Silo'][0],
        height -
        IT_Del_locations['Silo'][1],
        IT_Del_locations['Kaper'][0],
        height -
        IT_Del_locations['Kaper'][1],
        map_IT_Del.get('Silo', 'Kaper'))    
    make_line(
        city_map,
        IT_Del_locations['Silo'][0],
        height -
        IT_Del_locations['Silo'][1],
        IT_Del_locations['GD9'][0],
        height -
        IT_Del_locations['GD9'][1],
        map_IT_Del.get('Silo', 'GD9'))    

    #GD9
    make_line(
        city_map,
        IT_Del_locations['GD9'][0],
        height -
        IT_Del_locations['GD9'][1],
        IT_Del_locations['Kaper'][0],
        height -
        IT_Del_locations['Kaper'][1],
        map_IT_Del.get('GD9', 'Kaper')) 
    make_line(
        city_map,
        IT_Del_locations['GD9'][0],
        height -
        IT_Del_locations['GD9'][1],
        IT_Del_locations['KB'][0],
        height -
        IT_Del_locations['KB'][1],
        map_IT_Del.get('GD9', 'KB'))        

    #KB
    make_line(
        city_map,
        IT_Del_locations['KB'][0],
        height -
        IT_Del_locations['KB'][1],
        IT_Del_locations['Betfage'][0],
        height -
        IT_Del_locations['Betfage'][1],
        map_IT_Del.get('KB', 'Betfage'))  
    make_line(
        city_map,
        IT_Del_locations['KB'][0],
        height -
        IT_Del_locations['KB'][1],
        IT_Del_locations['GD8'][0],
        height -
        IT_Del_locations['GD8'][1],
        map_IT_Del.get('KB', 'GD8'))   

      #GD8
    make_line(
        city_map,
        IT_Del_locations['GD8'][0],
        height -
        IT_Del_locations['GD8'][1],
        IT_Del_locations['Kaper'][0],
        height -
        IT_Del_locations['Kaper'][1],
        map_IT_Del.get('GD8', 'Kaper'))  

    #Betfage
    make_line(
        city_map,
        IT_Del_locations['Betfage'][0],
        height -
        IT_Del_locations['Betfage'][1],
        IT_Del_locations['Perpus'][0],
        height -
        IT_Del_locations['Perpus'][1],
        map_IT_Del.get('Betfage', 'Perpus'))   

    #Audit
    make_line(
        city_map,
        IT_Del_locations['Audit'][0],
        height -
        IT_Del_locations['Audit'][1],
        IT_Del_locations['Perpus'][0],
        height -
        IT_Del_locations['Perpus'][1],
        map_IT_Del.get('Audit', 'Perpus'))  
    make_line(
        city_map,
        IT_Del_locations['Audit'][0],
        height -
        IT_Del_locations['Audit'][1],
        IT_Del_locations['EH'][0],
        height -
        IT_Del_locations['EH'][1],
        map_IT_Del.get('Audit', 'EH'))  

    # EH
    make_line(
        city_map,
        IT_Del_locations['EH'][0],
        height -
        IT_Del_locations['EH'][1],
        IT_Del_locations['Affair'][0],
        height -
        IT_Del_locations['Affair'][1],
        map_IT_Del.get('EH', 'Affair'))  
    make_line(
        city_map,
        IT_Del_locations['EH'][0],
        height -
        IT_Del_locations['EH'][1],
        IT_Del_locations['Circle'][0],
        height -
        IT_Del_locations['Circle'][1],
        map_IT_Del.get('EH', 'Circle'))   
    make_line(
        city_map,
        IT_Del_locations['EH'][0],
        height -
        IT_Del_locations['EH'][1],
        IT_Del_locations['Plaza'][0],
        height -
        IT_Del_locations['Plaza'][1],
        map_IT_Del.get('EH', 'Plaza'))

    #Plaza
    make_line(
        city_map,
        IT_Del_locations['Plaza'][0],
        height -
        IT_Del_locations['Plaza'][1],
        IT_Del_locations['OT'][0],
        height -
        IT_Del_locations['OT'][1],
        map_IT_Del.get('Plaza', 'OT')) 

    #OT
    make_line(
        city_map,
        IT_Del_locations['OT'][0],
        height -
        IT_Del_locations['OT'][1],
        IT_Del_locations['Surti'][0],
        height -
        IT_Del_locations['Surti'][1],
        map_IT_Del.get('OT', 'Surti'))    

    #Surti
    make_line(
        city_map,
        IT_Del_locations['Surti'][0],
        height -
        IT_Del_locations['Surti'][1],
        IT_Del_locations['Antiokia'][0],
        height -
        IT_Del_locations['Antiokia'][1],
        map_IT_Del.get('Surti', 'Antiokia')) 

    #Antiokia
    make_line(
        city_map,
        IT_Del_locations['Antiokia'][0],
        height -
        IT_Del_locations['Antiokia'][1],
        IT_Del_locations['Studio'][0],
        height -
        IT_Del_locations['Studio'][1],
        map_IT_Del.get('Antiokia', 'Studio'))  
    make_line(
        city_map,
        IT_Del_locations['Antiokia'][0],
        height -
        IT_Del_locations['Antiokia'][1],
        IT_Del_locations['GD7'][0],
        height -
        IT_Del_locations['GD7'][1],
        map_IT_Del.get('Antiokia', 'GD7'))  
    make_line(
        city_map,
        IT_Del_locations['Antiokia'][0],
        height -
        IT_Del_locations['Antiokia'][1],
        IT_Del_locations['GD5'][0],
        height -
        IT_Del_locations['GD5'][1],
        map_IT_Del.get('Antiokia', 'GD5'))   

    #Studio
    make_line(
        city_map,
        IT_Del_locations['Studio'][0],
        height -
        IT_Del_locations['Studio'][1],
        IT_Del_locations['GD7'][0],
        height -
        IT_Del_locations['GD7'][1],
        map_IT_Del.get('Studio', 'GD7'))

    #GD7
    make_line(
        city_map,
        IT_Del_locations['GD7'][0],
        height -
        IT_Del_locations['GD7'][1],
        IT_Del_locations['GD5'][0],
        height -
        IT_Del_locations['GD5'][1],
        map_IT_Del.get('GD7', 'GD5'))  
    make_line(
        city_map,
        IT_Del_locations['GD7'][0],
        height -
        IT_Del_locations['GD7'][1],
        IT_Del_locations['Satpam'][0],
        height -
        IT_Del_locations['Satpam'][1],
        map_IT_Del.get('GD7', 'Satpam'))   

    #GD4
    make_line(
        city_map,
        IT_Del_locations['GD4'][0],
        height -
        IT_Del_locations['GD4'][1],
        IT_Del_locations['Satpam'][0],
        height -
        IT_Del_locations['Satpam'][1],
        map_IT_Del.get('GD4', 'Satpam'))  
    make_line(
        city_map,
        IT_Del_locations['GD4'][0],
        height -
        IT_Del_locations['GD4'][1],
        IT_Del_locations['GD5'][0],
        height -
        IT_Del_locations['GD5'][1],
        map_IT_Del.get('GD4', 'GD5'))  
    make_line(
        city_map,
        IT_Del_locations['GD4'][0],
        height -
        IT_Del_locations['GD4'][1],
        IT_Del_locations['Affair'][0],
        height -
        IT_Del_locations['Affair'][1],
        map_IT_Del.get('GD4', 'Affair')) 
    make_line(
        city_map,
        IT_Del_locations['GD4'][0],
        height -
        IT_Del_locations['GD4'][1],
        IT_Del_locations['Circle'][0],
        height -
        IT_Del_locations['Circle'][1],
        map_IT_Del.get('GD4', 'Circle')) 
    make_line(
        city_map,
        IT_Del_locations['GD4'][0],
        height -
        IT_Del_locations['GD4'][1],
        IT_Del_locations['KL'][0],
        height -
        IT_Del_locations['KL'][1],
        map_IT_Del.get('GD4', 'KL'))      

    #Satpam
    make_line(
        city_map,
        IT_Del_locations['Satpam'][0],
        height -
        IT_Del_locations['Satpam'][1],
        IT_Del_locations['KL'][0],
        height -
        IT_Del_locations['KL'][1],
        map_IT_Del.get('Satpam', 'KL')) 

    # KL
    make_line(
        city_map,
        IT_Del_locations['KL'][0],
        height -
        IT_Del_locations['KL'][1],
        IT_Del_locations['Koperasi'][0],
        height -
        IT_Del_locations['Koperasi'][1],
        map_IT_Del.get('KL', 'Koperasi')) 
    make_line(
        city_map,
        IT_Del_locations['KL'][0],
        height -
        IT_Del_locations['KL'][1],
        IT_Del_locations['Circle'][0],
        height -
        IT_Del_locations['Circle'][1],
        map_IT_Del.get('KL', 'Circle')) 

    #Pniel
    make_line(
        city_map,
        IT_Del_locations['Pniel'][0],
        height -
        IT_Del_locations['Pniel'][1],
        IT_Del_locations['Koperasi'][0],
        height -
        IT_Del_locations['Koperasi'][1],
        map_IT_Del.get('Pniel', 'Koperasi')) 
    make_line(
        city_map,
        IT_Del_locations['Pniel'][0],
        height -
        IT_Del_locations['Pniel'][1],
        IT_Del_locations['PerDosen'][0],
        height -
        IT_Del_locations['PerDosen'][1],
        map_IT_Del.get('Pniel', 'PerDosen')) 

    #PerDosen     
    make_line(
        city_map,
        IT_Del_locations['PerDosen'][0],
        height -
        IT_Del_locations['PerDosen'][1],
        IT_Del_locations['Satpam1'][0],
        height -
        IT_Del_locations['Satpam1'][1],
        map_IT_Del.get('PerDosen', 'Satpam1')) 
    make_line(
        city_map,
        IT_Del_locations['PerDosen'][0],
        height -
        IT_Del_locations['PerDosen'][1],
        IT_Del_locations['Rektorat'][0],
        height -
        IT_Del_locations['Rektorat'][1],
        map_IT_Del.get('PerDosen', 'Rektorat')) 
   
    #Rektorat    
    make_line(
        city_map,
        IT_Del_locations['Rektorat'][0],
        height -
        IT_Del_locations['Rektorat'][1],
        IT_Del_locations['Satpam1'][0],
        height -
        IT_Del_locations['Satpam1'][1],
        map_IT_Del.get('PerDosen', 'Satpam1')) 


    for city in IT_Del_locations.keys():
        make_rectangle(
            city_map,
            IT_Del_locations[city][0],
            height -
            IT_Del_locations[city][1],
            margin,
            city)

    make_legend(city_map)


def make_line(map, x0, y0, x1, y1, distance):
    """This function draws out the lines joining various points."""
    map.create_line(x0, y0, x1, y1)
    map.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=distance)


def make_rectangle(map, x0, y0, margin, city_name):
    """This function draws out rectangles for various points."""
    global city_coord
    rect = map.create_rectangle(
        x0 - margin,
        y0 - margin,
        x0 + margin,
        y0 + margin,
        fill="white")
    if "jl_e" in city_name or "jl_parluasan" in city_name or "jl_y1" in city_name \
            or "jl_z" in city_name or "jl_y2" in city_name:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=E)
    else:
        map.create_text(
            x0 - 2 * margin,
            y0 - 2 * margin,
            text=city_name,
            anchor=SE)
    city_coord.update({city_name: rect})


def make_legend(map):

    map.create_rectangle(600, 100, 610, 110, fill="white")
    map.create_text(615, 105, anchor=W, text="Un-explored")

    map.create_rectangle(600, 115, 610, 125, fill="orange")
    map.create_text(615, 120, anchor=W, text="Frontier")

    map.create_rectangle(600, 130, 610, 140, fill="red")
    map.create_text(615, 135, anchor=W, text="Currently Exploring")

    map.create_rectangle(600, 145, 610, 155, fill="grey")
    map.create_text(615, 150, anchor=W, text="Explored")

    map.create_rectangle(600, 160, 610, 170, fill="dark green")
    map.create_text(615, 165, anchor=W, text="Final Solution")


def tree_search(problem):
    """
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Don't worry about repeated paths to a state. [Figure 3.7]
    This function has been changed to make it suitable for the Tkinter GUI.
    """
    global counter, frontier, node

    if counter == -1:
        frontier.append(Node(problem.initial))

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):

            return node
        frontier.extend(node.expand(problem))

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:

        display_explored(node)
    return None


def graph_search(problem):
    """
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Figure 3.7]
    This function has been changed to make it suitable for the Tkinter GUI.
    """
    global counter, frontier, node, explored
    if counter == -1:
        frontier.append(Node(problem.initial))
        explored = set()

        display_frontier(frontier)
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()

        display_current(node)
    if counter % 3 == 1 and counter >= 0:
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored and
                        child not in frontier)

        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def display_frontier(queue):
    """This function marks the frontier nodes (orange) on the map."""
    global city_map, city_coord
    qu = deepcopy(queue)
    while qu:
        node = qu.pop()
        for city in city_coord.keys():
            if node.state == city:
                city_map.itemconfig(city_coord[city], fill="orange")


def display_current(node):
    """This function marks the currently exploring node (red) on the map."""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="red")


def display_explored(node):
    """This function marks the already explored node (gray) on the map."""
    global city_map, city_coord
    city = node.state
    city_map.itemconfig(city_coord[city], fill="gray")


def display_final(cities):
    """This function marks the final solution nodes (green) on the map."""
    global city_map, city_coord
    for city in cities:
        city_map.itemconfig(city_coord[city], fill="green")


def best_first_graph_search(problem, f):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    global frontier, node, explored, counter

    if counter == -1:
        f = memoize(f, 'f')
        node = Node(problem.initial)
        display_current(node)
        if problem.goal_test(node.state):
            return node
        frontier = PriorityQueue('min', f)
        frontier.append(node)
        display_frontier(frontier)
        explored = set()
    if counter % 3 == 0 and counter >= 0:
        node = frontier.pop()
        display_current(node)
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
    if counter % 3 == 1 and counter >= 0:
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child]
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
        display_frontier(frontier)
    if counter % 3 == 2 and counter >= 0:
        display_explored(node)
    return None


def uniform_cost_search(problem):
    """[Figure 3.14]"""
    return best_first_graph_search(problem, lambda node: node.path_cost)

# TODO:
# Remove redundant code.
# Make the interchangbility work between various algorithms at each step.
def on_click():
    """
    This function defines the action of the 'Next' button.
    """
    global algo, counter, next_button, IT_Del_problem, start, goal
    IT_Del_problem = GraphProblem(start.get(), goal.get(), map_IT_Del)
    if "Uniform Cost Search" == algo.get():
        node = uniform_cost_search(IT_Del_problem)
        if node is not None:
            final_path = ucs(IT_Del_problem).solution()
            final_path.append(start.get())

            show_cost_and_path(final_path, ucs(IT_Del_problem).path_cost)

            display_final(final_path)
            next_button.config(state="disabled")
        counter += 1


def reset_map():
    global counter, city_coord, city_map, next_button
    counter = -1
    for city in city_coord.keys():
        city_map.itemconfig(city_coord[city], fill="white")
    next_button.config(state="normal")
    # clear_widget()
    # show_cost_and_path(path_lbl, "", "")

def show_cost_and_path(path, cost):   
    # clear_widget(path_lbl)
    path_lbl = Label(root, text = "Path : "+ str(path))
    cost_lbl = Label(root, text = "Cost : "+ str(cost))
    
    path_lbl.place(x=700, y=125)
    cost_lbl.place(x=700, y=150)
    
def clear_widget(widget):
    widget.destroy()


# TODO: Add more search algorithms in the OptionMenu


def main():
    
    global algo, start, goal, next_button

    #menggunakan tkinter untuk membuat windowsnya
    root = Tk()
    #judul dari windows
    root.title("Route finding using UCS")
    
    #panjang x lebar windows app
    root.geometry("1000x800") 


    algo = StringVar(root)
    start = StringVar(root)
    goal = StringVar(root)


    algo.set("Uniform Cost Search")

    Label(root, text="\n Search Algorithm : " + algo.get()).pack()
    start.set('GD9')
    goal.set('GD5')


    #menyimpan lokasi di cities, diambil dari keys dari setiap location yang ada di objek map_IT_Del
    cities = sorted(map_IT_Del.locations.keys())

    #isi dari algorithm menu
    # algorithm_menu = OptionMenu(
    #     root,
    #     algo, "Ubah di kode","Uniform Cost Search")

    
    # Label(root, text="\n Search Algorithm").pack()
    # algorithm_menu.pack()

    
    path_lbl = Label(root, text = "Path : ")
    cost_lbl = Label(root, text = "Cost : ")
    path_lbl.place(x=700, y=125)
    cost_lbl.place(x=700, y=150)


    Label(root, text="\n Awal").pack()
    start_menu = OptionMenu(root, start, *cities)
    start_menu.pack()
    Label(root, text="\n Goal").pack()
    goal_menu = OptionMenu(root, goal, *cities)
    goal_menu.pack()
    frame1 = Frame(root)
    next_button = Button(
        frame1,
        width=4,
        height=1,
        text="Next",
        command=on_click,
        padx=2,
        pady=2,
        relief=GROOVE)
    next_button.pack(side=RIGHT)
    reset_button = Button(
        frame1,
        width=4,
        height=1,
        text="Reset",
        command=reset_map,
        padx=2,
        pady=2,
        relief=GROOVE)
    reset_button.pack(side=RIGHT)
    frame1.pack(side=BOTTOM)

    create_map(root)

    root.mainloop()


if __name__ == "__main__":
    main()