import tkinter as tk
from tkinter import simpledialog
from PIL import Image, ImageTk, ImageDraw, ImageFont
import networkx as nx
from IPython import embed
import argparse
from PIL import Image
import os 

class SceneGraphBuilder:
    def __init__(self, root, image_path,zoomed_img_size,out_path,node_types):
        self.root = root
        self.root.title("Scene Graph Builder")
        self.zoomed_img_size = zoomed_img_size
        # Load image
        self.image = Image.open(image_path)
        print(self.image.size)
        self.image_tk = ImageTk.PhotoImage(self.image)

        # Create canvas for image display
        self.canvas = tk.Canvas(root, width=self.image.size[0], height=self.image.size[1])
        self.canvas.pack()

        # Display image on canvas
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image_tk)
        self.output_image = self.image.copy()
        self.output_image_draw = ImageDraw.Draw(self.output_image)
        
        # Initialize NetworkX graph
        self.graph = nx.Graph()

        # Node tracking
        self.nodes = {}
        self.node_types = node_types
        self.edges = []

        # Bind mouse events
        self.canvas.bind("<Double-Button-1>", self.add_node)
        self.canvas.bind("<Button-1>", self.start_edge)
        self.canvas.bind("<B1-Motion>", self.drag_edge)
        self.canvas.bind("<ButtonRelease-1>", self.end_edge)
        root.bind("<Return>",self.save_canvas_as_img)
        self.node_number = 1
        self.fine_node_counter = 64
        self.current_start_node = None
        self.zoomed_images = {}
        self.zoomed_images_draw = {}
        self.img_save_path = out_path
        
        self.zimgfont = "nimbus"
        self.pimgfont = "nimbus"
        
        self.zoomed_oval_size = 10
        self.parent_oval_size = 15
        
        self.zoomed_imgfontsize = 20
        self.parent_imgfontsize = 30
        self.zoomed_imgfont = ImageFont.truetype("arial.ttf", self.zoomed_imgfontsize)
        self.parent_imgfont = ImageFont.truetype("arial.ttf", self.parent_imgfontsize)

    def save_canvas_as_img(self, event):
        print("Saving image...")
        self.output_image.save(os.path.join(self.img_save_path,'scene_graph.png'))
        for k,v in self.zoomed_images.items():
            v.save(os.path.join(self.img_save_path,f'scene_graph_{k}.png'))    
    
    def add_node(self, event):
        x, y = event.x, event.y
        dialog_msg = """\nEnter the node's type: \n"""
        for i,n in enumerate(self.node_types):
            dialog_msg += f"\n{i+1}:{n}"
        
        node_type = simpledialog.askinteger("Node Type", dialog_msg)

        if node_type and 0<node_type<len(self.node_types)+1:
            # Add node to graph
            self.graph.add_node(self.node_number, type=node_type, pos=(x, y))
            self.nodes[self.node_number] = (x, y)

            # Display node visually
            self.canvas.create_oval(x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size, fill="red")
            self.canvas.create_text(x, y, text=self.node_number,font=(self.pimgfont, self.parent_imgfontsize), fill="black")
            
            self.output_image_draw.ellipse((x - self.parent_oval_size, y - self.parent_oval_size, x + self.parent_oval_size, y + self.parent_oval_size), fill=(255,0,0))
            self.output_image_draw.text((x, y), str(self.node_number), fill=(0,0,0),font=self.parent_imgfont)
            
            self.open_zoomed_window(x, y, self.node_number)
            self.node_number+=1
    
    def open_zoomed_window(self, x, y, parent_node_name):
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
        
        self.zoomed_images[parent_node_name] = cropped_image.copy()
        self.zoomed_images_draw[parent_node_name] = ImageDraw.Draw(self.zoomed_images[parent_node_name])
        
        zoom_canvas.create_image(0, 0, anchor=tk.NW, image=zoomed_image)
        zoom_canvas.image = zoomed_image  # keep a reference!
        self.fine_node_counter = 64
        
        #create parent circle and text in zoomed window 
        zoom_canvas.create_oval(x-left-self.zoomed_oval_size, y-upper-self.zoomed_oval_size ,x-left + self.zoomed_oval_size, y-upper + self.zoomed_oval_size, fill="red")
        zoom_canvas.create_text(x-left, y-upper, text=parent_node_name, fill="black",font=(self.pimgfont,self.zoomed_imgfontsize))
        
        #add the circle and text to zoomed images
        self.zoomed_images_draw[parent_node_name].ellipse((x-left-self.zoomed_oval_size, y-upper-self.zoomed_oval_size, x-left+self.zoomed_oval_size, y-upper+self.zoomed_oval_size), fill=(255,0,0))
        self.zoomed_images_draw[parent_node_name].text((x-left, y-upper), str(parent_node_name), fill=(0,0,0),font=self.zoomed_imgfont)
        
        zoom_canvas.bind("<Button-1>", lambda event, pn=parent_node_name: self.add_child_node(event, zoom_canvas, pn, left, upper,x-left,y-upper))
        
    def add_child_node(self, event, canvas, parent_node_name, offset_x, offset_y,px,py):
        self.fine_node_counter+=1
        x, y = event.x, event.y
        # Calculate global coordinates
        global_x, global_y = x + offset_x, y + offset_y
        self.graph.add_node(chr(self.fine_node_counter), type="child", pos=(global_x, global_y))
        self.graph.add_edge(parent_node_name, chr(self.fine_node_counter))
        print(self.fine_node_counter)
        
        # Display node on the zoomed canvas and main canvas
        canvas.create_oval(x - self.zoomed_oval_size, y - self.zoomed_oval_size, x + self.zoomed_oval_size, y + self.zoomed_oval_size, fill="green")
        canvas.create_text(x, y, text=chr(self.fine_node_counter), fill="black",font=(self.zimgfont, self.zoomed_imgfontsize))
        canvas.create_line([px,py, x, y], fill="green")
        
        # Display child nodes in the zoomed image
        self.zoomed_images_draw[parent_node_name].ellipse((x-self.zoomed_oval_size, y-self.zoomed_oval_size, x+self.zoomed_oval_size, y+self.zoomed_oval_size), fill=(0,255,0))
        self.zoomed_images_draw[parent_node_name].text((x, y), str(chr(self.fine_node_counter)), fill=(0,0,0),font=self.zoomed_imgfont)
        
    def start_edge(self, event):
        self.current_start_node = self.get_nearest_node(event.x, event.y)

    def drag_edge(self, event):
        if self.current_start_node:
            self.canvas.delete("temp_edge")
            self.canvas.create_line(*self.nodes[self.current_start_node], event.x, event.y, tags="temp_edge", dash=(2, 2),width=5)
    
    def end_edge(self, event):
        self.canvas.delete("temp_edge")
        if self.current_start_node:
            end_node = self.get_nearest_node(event.x, event.y)
            if end_node and end_node != self.current_start_node:
                self.graph.add_edge(self.current_start_node, end_node)
                self.edges.append((self.current_start_node, end_node))
                
                # Display edge visually
                self.canvas.create_line(*self.nodes[self.current_start_node], *self.nodes[end_node], fill="blue")
                self.output_image_draw.line([*self.nodes[self.current_start_node], event.x, event.y], fill=(0,0,255), width=5)
                print([*self.nodes[self.current_start_node], event.x, event.y])
            self.current_start_node = None

    def get_nearest_node(self, x, y):
        min_distance = float('inf')
        nearest_node = None

        for name, (nx, ny) in self.nodes.items():
            distance = ((nx - x) ** 2 + (ny - y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_node = name

        return nearest_node

    def get_graph(self):
        return self.graph
    
    def show_graph(self):
        print(nx.info(self.graph))

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--img", required=True, help="path to image file")
    parser.add_argument("--out", required=True, help="output folder") 
    args = parser.parse_args()
    
    root = tk.Tk()
    fine_root = tk.Tk()
    zoomed_img_size = 250
    node_types = [
         'INTERSECTION',
         'PASSAGEWAY',
         'DOORWAY',
         'LIFT/STAIRS',
         'DOOR',
         'OPEN AREA',
         'ROOM',
         'CORNER']
    print("""
          -------------------CONTROLS--------------------------------------
          DOUBLE CLICK TO ADD A NODE (THEN SELECT THE NODE TYPE)
          CLICK AND DRAG BETWEEN 2 NODES TO ADD AN EDGE.
          PRESS RETURN KEY TO SAVE ALL THE IMAGES
          ----ADD NODES FOR THE FOLLOWING REGIONS IN YOUR MAP--------------
          1. INTERSECTION
          2. PASSAGEWAY (ALSO ADD NODES ALONG THE PASSAGEWAY)
          3. DOORWAY
          4. LIFT/STAIRS
          5. DOOR
          6. OPEN AREA
          7. ROOM
          8. CORNER
          
          ONCE YOU'VE DOUBLE CLICKED TO ADD A NODE, YOU CAN ADD ADDITIONAL 'FINE NODES' AROUND THE 
          NODE TO GIVE THE LLM BETTER CONTROL OVER THE SCENARIO. BE CAREFUL TO NOT OVERLAP THESE WITH EXISTING NODES.
          ---------------------------------------------------------------------
          """)
    app = SceneGraphBuilder(root, args.img,zoomed_img_size,args.out,node_types)  # Replace with your own image path
    root.mainloop()
    graph = app.get_graph()
    graph = nx.Graph(graph).to_undirected()
    serialized_graph = nx.node_link_data(graph)
    undirected_links = [f"{link['source']}<->{link['target']}" for link in serialized_graph['links']]
    serialized_graph['links'] = undirected_links
    del serialized_graph['graph']
    del serialized_graph['directed']
    del serialized_graph['multigraph']
    print(serialized_graph)
    embed()
