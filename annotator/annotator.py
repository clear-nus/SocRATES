import tkinter as tk
from tkinter import ttk
from tkinter import simpledialog
from PIL import Image, ImageTk, ImageDraw, ImageFont
import networkx as nx
from IPython import embed
import argparse
from PIL import Image
import os 
import json
import secrets
import numpy as np
from omegaconf import OmegaConf

class SceneGraphBuilder:
    def __init__(self, root, zoom_in,zoom_out, image_path, scgraph_path, out_path, node_types, edge_types):
        self.root = root
        self.root.title("Scene Graph Builder")
        # Load image
        self.image = Image.open(image_path).convert('RGB')
        width,height = self.image.size
        self.zoom = zoom_in
        self.zoom_out = zoom_out
        self.unscaled_image = self.image.copy()
        self.image = self.image.resize((round(width*self.zoom),round(height*self.zoom)))
        print(self.image.size)
        self.image_tk = ImageTk.PhotoImage(self.image)
        # Create canvas for image display
        # vbar = tk.Scrollbar(self.root,orient=tk.VERTICAL)
        # hbar = tk.Scrollbar(self.root,orient=tk.HORIZONTAL)
        # vbar.grid(row=0, column=1, sticky='ns')
        # hbar.grid(row=1, column=0, sticky='we')
        self.canvas = tk.Canvas(root, width=self.image.size[0], height=self.image.size[1])#,xscrollcommand=hbar.set,yscrollcommand=vbar.set)
        # self.canvas.grid(row=0, column=0, sticky='nswe')
        # self.canvas.update()
        # vbar.configure(command = self.scroll_x)
        # hbar.configure(command = self.scroll_y)
        self.canvas.pack()

        # Display image on canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image_tk)
        self.output_image = self.unscaled_image.resize((round(width*self.zoom_out),round(height*self.zoom_out))).copy()
        self.output_image_draw = ImageDraw.Draw(self.output_image,mode='RGBA')
        print(self.output_image_draw.mode)
        # Initialize NetworkX graph
        self.graph = nx.Graph()

        # Node tracking
        self.nodes = {}
        self.node_types = node_types
        self.edge_types = edge_types
        self.edges = []

        # Bind mouse events
        self.canvas.bind("<Double-Button-1>", self.add_node)
        self.canvas.bind("<Button-1>", self.start_edge)
        self.canvas.bind("<B1-Motion>", self.drag_edge)
        self.canvas.bind("<ButtonRelease-1>", self.end_edge)
        #self.canvas.bind("<ButtonPress-3>",self.move_from)
        #self.canvas.bind("<B3-Motion>",self.move_to)
        root.bind("<Alt-s>",self.save_canvas_as_img)
        self.fine_node_counter = 64
        self.current_start_node = None
        self.zoomed_images = {}
        self.zoomed_images_draw = {}
        self.img_save_path = out_path
        self.image_origin = np.array([0.0,0.0])
        self.zimgfont = "nimbus"
        self.pimgfont = "nimbus"
        
        self.parent_oval_size = round(20*self.zoom_out/self.zoom)
        self.parent_imgfontsize = round(20*self.zoom_out/self.zoom)
        self.export_imgfontsize = round(30*self.zoom_out/self.zoom)
        self.parent_imgfont = ImageFont.truetype("arial.ttf", self.export_imgfontsize)
        self.node_names = []
        self.scgraph_save_path = scgraph_path
    def save_canvas_as_img(self, event):
        print("Saving image...")
        self.output_image.save(os.path.join(self.img_save_path))
        for k,v in self.zoomed_images.items():
            v.save(os.path.join(self.img_save_path,f'{k}.png')) 
        
        graph = self.graph.copy()
        graph = nx.Graph(graph).to_undirected()
        serialized_graph = nx.node_link_data(graph)
        print(serialized_graph)
        with open(os.path.join(self.scgraph_save_path), "w") as f:
            json.dump(serialized_graph, f, indent=4)   
    
    def add_node(self, event):
        #print("Add Node")
        x, y = event.x, event.y
        dialog_msg = """\nEnter the node's type: \n"""
        for i,n in enumerate(self.node_types):
            dialog_msg += f"\n{i+1}:{n}"
        dialog_msg+='\n'
        node_name = None #input("Enter the node's name, e.g. 'kitchen' (optional)\n")#simpledialog.askstring("Node Name", "Enter the node's name, e.g. 'Kitchen' (optional):")
        while True:
            try:
                node_type = int(input(dialog_msg))#simpledialog.askinteger("Node Type", dialog_msg)
                print(node_type)
                break
            except ValueError:
                print("Invalid node type, try again")
                
        
        random_node_name = ''
        if node_name == None:
            node_name = ''
            while True:
                random_node_name = secrets.token_hex(1) #random 4 digit alphanumeric
                if random_node_name not in self.node_names:
                    self.node_names.append(random_node_name)
                    break
        
        full_node_name = random_node_name + node_name
        if node_type and 0<node_type<len(self.node_types)+1:
            # Add node to graph
            print((x,y))
            self.graph.add_node(full_node_name, type=self.node_types[node_type-1], pos=(round(x/self.zoom), round(y/self.zoom)))
            
            self.nodes[full_node_name] = (x, y)

            # Display node visually
            #self.canvas.create_oval(x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size, fill="red")
            self.canvas.create_rectangle(x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size, fill='yellow', width = 5, outline="blue")
            #self.canvas.create_text(x-0.5, y-0.5, text=full_node_name,font=(self.pimgfont, self.parent_imgfontsize), fill="black")
            self.canvas.create_text(x, y, text=full_node_name,font=(self.pimgfont, self.parent_imgfontsize), fill="black")
            print(f'x-:{x - self.parent_oval_size}, y-:{y - self.parent_oval_size}')
            print(f'x+:{x + self.parent_oval_size}, y+:{y + self.parent_oval_size}')
            print(f'full node name:{full_node_name}')
            
            self.output_image_draw.rectangle(
                (
                 round((x - self.parent_oval_size)*self.zoom_out/self.zoom), round((y - self.parent_oval_size)*self.zoom_out/self.zoom), 
                 round((x + self.parent_oval_size)*self.zoom_out/self.zoom), round((y + self.parent_oval_size)*self.zoom_out/self.zoom)
                ),fill='yellow',width=round(5*self.zoom_out/self.zoom),outline='blue'
                )
            self.output_image_draw.text(
                (
                    round(x*self.zoom_out/self.zoom), round(y*self.zoom_out/self.zoom)
                ), full_node_name,anchor='mm', fill='black',font=self.parent_imgfont)
            
    def start_edge(self, event):
        #print("Start Edge")
        self.current_start_node = self.get_nearest_node(event.x, event.y)

    def drag_edge(self, event):
        #print("Drag Edge")
        if self.current_start_node:
            self.canvas.delete("temp_edge")
            self.canvas.create_line(*self.nodes[self.current_start_node], event.x, event.y, tags="temp_edge", dash=(2, 2),width=5)
    
    def end_edge(self, event):
        #print("End Edge")
        self.canvas.delete("temp_edge")
        edge_type = None
        if self.current_start_node:
            end_node = self.get_nearest_node(event.x, event.y)
            if end_node and end_node != self.current_start_node:
                
                dialog_msg = """\nEnter the Edge's type: \n"""
                for i,n in enumerate(self.edge_types):
                    dialog_msg += f"\n{i+1}:{n}"
                dialog_msg+='\n'
                #edge_type = simpledialog.askinteger("Node Type", dialog_msg)
                edge_type = int(input(dialog_msg))
                print(edge_type)
                print(self.edge_types[edge_type-1])
                self.graph.add_edge(self.current_start_node, end_node,type = self.edge_types[edge_type-1])
                self.edges.append((self.current_start_node, end_node))
                
                current_start_node = self.nodes[self.current_start_node]
                # Display edge visually
                self.canvas.create_line(*current_start_node, *self.nodes[end_node], fill="blue")
                self.output_image_draw.line([round(current_start_node[0]*self.zoom_out/self.zoom),round(current_start_node[1]*self.zoom_out/self.zoom), round(self.nodes[end_node][0]*self.zoom_out/self.zoom), round(self.nodes[end_node][1]*self.zoom_out/self.zoom)], fill="red", width=round(2*self.zoom_out/self.zoom))
                
                #print([*self.nodes[self.current_start_node], event.x, event.y])
            self.current_start_node = None
            self.current_start_node = None
            
            self.current_start_node = None          
            
    def move_from(self, event):
        ''' Remember previous coordinates for scrolling with the mouse '''
        self.image_origin[0]-=event.x
        self.image_origin[1]-=event.y

    def move_to(self, event):
        ''' Drag (move) canvas to the new position '''
        self.canvas.scan_dragto(event.x, event.y, gain=1)
        #self.show_image()  # redraw the image

    def get_nearest_node(self, x, y):
        #print("Getting nearest Node")
        min_distance = float('inf')
        nearest_node = None

        for name, (nx, ny) in self.nodes.items():
            distance = ((nx - x) ** 2 + (ny - y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_node = name
        #print(f'Nearest Node:{nearest_node}')
        return nearest_node 
    
    def show_graph(self):
        print(nx.info(self.graph))

if __name__ == "__main__":
    
    config = OmegaConf.load('config.yaml')   
    root = tk.Tk()
    fine_root = tk.Tk()
    
    msg = f"""
          -------------------CONTROLS--------------------------------------
          DOUBLE CLICK TO ADD A NODE (THEN SELECT THE NODE TYPE)
          CLICK AND DRAG BETWEEN 2 NODES TO ADD AN EDGE.
          PRESS RETURN KEY TO SAVE ALL THE IMAGES
          ----ADD NODES FOR THE FOLLOWING REGIONS IN YOUR MAP--------------""" +'\n'+ '\n'.join([f"{i+1}:{n}" for i,n in enumerate(config['node_types'])]) + '\n'+"""-- CONNECT THE NODES WITH EDGES OF THE FOLLOWING TYPES----------"""+ '\n'+'\n'.join([f"{i+1}:{n}" for i,n in enumerate(config['edge_types'])]) + '\n'+"""--------------------------------------------------------------------"""
    print(msg)
    app = SceneGraphBuilder(root, float(config['zoom_in']), float(config['zoom_out']),config['img'],config['scg'],config['out'],config['node_types'],config['edge_types'])  # Replace with your own image path
    root.mainloop()
