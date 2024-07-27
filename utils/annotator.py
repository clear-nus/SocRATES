import tkinter as tk
from tkinter import simpledialog
from PIL import Image, ImageTk, ImageDraw, ImageFont
import networkx as nx
from IPython import embed
import argparse
from PIL import Image
import os 
import json
import secrets

class SceneGraphBuilder:
    def __init__(self, root, image_path,img_zoom,img_rotate,out_path,node_types, edge_types):
        self.root = root
        self.root.title("Scene Graph Builder")
        self.zoomed_img_size = img_zoom
        # Load image
        image = Image.open(image_path).convert('RGB')
        print(image.size)
        self.image = image.rotate(img_rotate, expand=True)
        print(image.size)
        width,height = self.image.size
        self.zoom = img_zoom
        self.image = self.image.resize((round(width*self.zoom),round(height*self.zoom)))
        print(self.image.size)
        self.image_tk = ImageTk.PhotoImage(self.image)
        # Create canvas for image display
        self.canvas = tk.Canvas(root, width=self.image.size[0], height=self.image.size[1])
        self.canvas.pack()

        # Display image on canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image_tk)
        self.output_image = self.image.copy()
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
        root.bind("<Alt-s>",self.save_canvas_as_img)
        self.fine_node_counter = 64
        self.current_start_node = None
        self.zoomed_images = {}
        self.zoomed_images_draw = {}
        self.img_save_path = out_path
        
        self.zimgfont = "nimbus"
        self.pimgfont = "nimbus"
        
        self.zoomed_oval_size = 10
        self.parent_oval_size = 40
        
        self.zoomed_imgfontsize = 20
        self.parent_imgfontsize = 40
        self.zoomed_imgfont = ImageFont.truetype("arial.ttf", self.zoomed_imgfontsize)
        self.parent_imgfont = ImageFont.truetype("arial.ttf", self.parent_imgfontsize)
        self.node_names = []

    def save_canvas_as_img(self, event):
        print("Saving image...")
        self.output_image.save(os.path.join(self.img_save_path,'scene_graph.png'))
        for k,v in self.zoomed_images.items():
            v.save(os.path.join(self.img_save_path,f'{k}.png')) 
        
        graph = self.graph.copy()
        graph = nx.Graph(graph).to_undirected()
        serialized_graph = nx.node_link_data(graph)
        #undirected_links = [f"{link['source']}<->{link['target']}" for link in serialized_graph['links']]
        #serialized_graph['links'] = undirected_links
        #del serialized_graph['graph']
        #del serialized_graph['directed']
        #del serialized_graph['multigraph']
        print(serialized_graph)
        with open(os.path.join(args.out,"scene_graph.json"), "w") as f:
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
            self.graph.add_node(full_node_name, type=self.node_types[node_type-1], pos=(x, y))
            
            self.nodes[full_node_name] = (x, y)

            # Display node visually
            #self.canvas.create_oval(x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size, fill="red")
            self.canvas.create_rectangle(x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size, fill='', width = 5, outline="blue")
            #self.canvas.create_text(x-0.5, y-0.5, text=full_node_name,font=(self.pimgfont, self.parent_imgfontsize), fill="black")
            self.canvas.create_text(x, y, text=full_node_name,font=(self.pimgfont, self.parent_imgfontsize), fill="black")
            print(f'x-:{x - self.parent_oval_size}, y-:{y - self.parent_oval_size}')
            print(f'x+:{x + self.parent_oval_size}, y+:{y + self.parent_oval_size}')
            print(f'full node name:{full_node_name}')
            
            #self.output_image_draw.ellipse((x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size), fill=(255,0,0))
            self.output_image_draw.rectangle((x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size),fill=None,width=5,outline='blue')
            #self.output_image_draw.text((x-0.5, y-0.5), full_node_name, fill=(0,0,0),font=self.parent_imgfont)
            self.output_image_draw.text((x, y), full_node_name,anchor='mm', fill='black',font=self.parent_imgfont)
            
            #self.open_zoomed_window(x, y, str(self.node_number)) no more child nodes
    '''
    def open_zoomed_window(self, x, y, parent_node_number):
        print("Opening Zoomed window")
        zoom_window = tk.Toplevel(self.root)
        zoom_window.title("Add Child Nodes")
        zoom_canvas = tk.Canvas(zoom_window, width=self.zoomed_img_size, height=self.zoomed_img_size)
        zoom_canvas.pack()
        # Compute the crop area
        left = max(0, x - self.zoomed_img_size // 2)
        upper = max(0, y - self.zoomed_img_size // 2)
        right = min(self.image.size[0], x + self.zoomed_img_size // 2)
        lower = min(self.image.size[1], y + self.zoomed_img_size // 2)

        # Extract and display zoomed image portion
        cropped_image = self.image.crop((left, upper, right, lower))
        zoomed_image = ImageTk.PhotoImage(cropped_image) 
        
        self.zoomed_images[parent_node_number] = cropped_image.copy()
        self.zoomed_images_draw[parent_node_number] = ImageDraw.Draw(self.zoomed_images[parent_node_number])
        
        zoom_canvas.create_image(0, 0, anchor=tk.NW, image=zoomed_image)
        zoom_canvas.image = zoomed_image  # keep a reference!
        self.fine_node_counter = 64
        
        #create parent circle and text in zoomed window 
        zoom_canvas.create_oval(x-left-self.zoomed_oval_size, y-upper-self.zoomed_oval_size ,x-left + self.zoomed_oval_size, y-upper + self.zoomed_oval_size, fill="red")
        zoom_canvas.create_text(x-left, y-upper, text=parent_node_number, fill="black",font=(self.pimgfont,self.zoomed_imgfontsize))
        
        #add the circle and text to zoomed images
        self.zoomed_images_draw[parent_node_number].ellipse((x-left-self.zoomed_oval_size, y-upper-self.zoomed_oval_size, x-left+self.zoomed_oval_size, y-upper+self.zoomed_oval_size), fill=(255,0,0))
        self.zoomed_images_draw[parent_node_number].text((x-left, y-upper), str(parent_node_number), fill=(0,0,0),font=self.zoomed_imgfont)
        
        zoom_canvas.bind("<Button-1>", lambda event, pn=parent_node_number: self.add_child_node(event, zoom_canvas, pn, left, upper,x-left,y-upper))
        
    def add_child_node(self, event, canvas, parent_node_number, offset_x, offset_y,px,py):
        print("Adding child node")
        print(self.nodes)
        self.fine_node_counter+=1
        x, y = event.x, event.y
        # Calculate global coordinates
        global_x, global_y = x + offset_x, y + offset_y
        self.graph.add_node(str(parent_node_number)+chr(self.fine_node_counter),name='', type="child", pos=(global_x, global_y))
        print(self.graph.nodes)
        self.graph.add_edge(str(parent_node_number), str(parent_node_number)+chr(self.fine_node_counter))
        print(self.fine_node_counter)
        
        # Display node on the zoomed canvas and main canvas
        canvas.create_oval(x - self.zoomed_oval_size, y - self.zoomed_oval_size, x + self.zoomed_oval_size, y + self.zoomed_oval_size, fill="green")
        canvas.create_text(x, y, text=chr(self.fine_node_counter), fill="black",font=(self.zimgfont, self.zoomed_imgfontsize))
        canvas.create_line([px,py, x, y], fill="green")
        
        # Display child nodes in the zoomed image
        self.zoomed_images_draw[parent_node_number].ellipse((x-self.zoomed_oval_size, y-self.zoomed_oval_size, x+self.zoomed_oval_size, y+self.zoomed_oval_size), fill=(0,255,0))
        self.zoomed_images_draw[parent_node_number].text((x, y), str(chr(self.fine_node_counter)), fill=(0,0,0),font=self.zoomed_imgfont)
    '''
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
                print(self.edge_types[edge_type])
                self.graph.add_edge(self.current_start_node, end_node,type = self.edge_types[edge_type-1])
                self.edges.append((self.current_start_node, end_node))
                
                current_start_node = self.nodes[self.current_start_node]
                # Display edge visually
                self.canvas.create_line(*current_start_node, *self.nodes[end_node], fill="blue")
                self.output_image_draw.line([current_start_node[0],current_start_node[1], event.x, event.y], fill="red", width=3)
                
                #print([*self.nodes[self.current_start_node], event.x, event.y])
            self.current_start_node = None
            self.current_start_node = None
            
            self.current_start_node = None          
            

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
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--img", required=True, help="path to image file")
    parser.add_argument("--out", required=True, help="output folder") 
    args = parser.parse_args()
    
    root = tk.Tk()
    fine_root = tk.Tk()
    img_zoom = 1.0
    img_rotate= 0
    node_types = [
         'INTERSECTION',
         'LIFT',
         'OPEN AREA',
         'NARROW AISLE',
         'MID-PASSAGEWAY',
         'END-PASSAGEWAY',
         'ROOM',
         'CORNER']
    edge_types = [
        'DOORWAY',
        'NARROW DOORWAY',
        'PASSAGEWAY',
        'NARROW PASSAGEWAY',
        'STAIRS',
        'ROOM ENTRANCE/EXIT'
    ]
    msg = f"""
          -------------------CONTROLS--------------------------------------
          DOUBLE CLICK TO ADD A NODE (THEN SELECT THE NODE TYPE)
          CLICK AND DRAG BETWEEN 2 NODES TO ADD AN EDGE.
          PRESS RETURN KEY TO SAVE ALL THE IMAGES
          ----ADD NODES FOR THE FOLLOWING REGIONS IN YOUR MAP--------------""" +'\n'+ '\n'.join([f"{i+1}:{n}" for i,n in enumerate(node_types)]) + '\n'+"""-- CONNECT THE NODES WITH EDGES OF THE FOLLOWING TYPES----------"""+ '\n'+'\n'.join([f"{i+1}:{n}" for i,n in enumerate(edge_types)]) + '\n'+"""--------------------------------------------------------------------"""
    print(msg)
    app = SceneGraphBuilder(root, args.img,img_zoom,img_rotate,args.out,node_types,edge_types)  # Replace with your own image path
    root.mainloop()
