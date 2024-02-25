import osmnx as ox
import requests
import pandas as pd

import pyodbc

conn = pyodbc.connect('Driver={SQL Server};'
                      'Server=localhost\SQLEXPRESS;'
                      'Database=GridDataDB;'
                      'Trusted_Connection=yes;')
cursor = conn.cursor()

def get_elevation_open_elevation(lat, lon):
    base_url = "https://api.open-elevation.com/api/v1/lookup"
    params = {
        "locations": f"{lat},{lon}",
    }

    response = requests.get(base_url, params=params)
    data = response.json() if response is not None and response.content else None

    if response is None or not response.content:
        print("empty response")
        print(params)

    if "results" in data and data["results"]:
        elevation = data["results"][0]["elevation"]
        return elevation
    else:
        return None
    print("import")

def create_table_node():
    cursor.execute('''
		CREATE TABLE Node (
            Id UNIQUEIDENTIFIER PRIMARY KEY,
            GeographyValues GEOGRAPHY,
            IncidentEdge UNIQUEIDENTIFIER FOREIGN KEY REFERENCES Edge(Id)
            )
                ''')

def create_table_edge():
    cursor.execute('''
		CREATE TABLE Edge (
            Id UNIQUEIDENTIFIER PRIMARY KEY,
            ShoelaceForward DECIMAL(18, 0),
            ShoelaceBackward DECIMAL(18, 0),
            Tags TEXT,
            GeoLocations GEOGRAPHY,
            SourceNode UNIQUEIDENTIFIER FOREIGN KEY REFERENCES Edge(Id),
            TargetNode UNIQUEIDENTIFIER FOREIGN KEY REFERENCES Edge(Id),
            Reversed BIT NULL,
            Cost DECIMAL(18, 0),
            OneWay BIT NULL            
			)
               ''')
    

# Specify the place name for North Rhine-Westphalia, Germany
place_name = "Oespel, Dortmund, North-Rhine Westphalia, Germany"

# Create the graph for NRW
G = ox.graph_from_place(place_name, network_type='all')
print("graph")
print(G)

# Add elevation data to nodes
for node, data in G.nodes(data=True):
    elevation = get_elevation_open_elevation(data['y'], data['x'])
    data['elevation'] = elevation

print("elevation")


# Convert the graph data to pandas DataFrames
df_nodes, df_edges = ox.graph_to_gdfs(G)
print("convert")


# Save the DataFrames to CSV files
df_nodes.to_csv('nodes_nrw.csv', index=False)
df_edges.to_csv('edges_nrw.csv', index=False)


G = ox.elevation.add_edge_grades(G)
nc = ox.plot.get_node_colors_by_attr(G, "elevation", cmap="plasma")
fig, ax = ox.plot_graph(G, node_color=nc, node_size=20, edge_linewidth=2, edge_color="#333")

print("done")


# Query to get all tables
tables = cursor.execute("SELECT TABLE_NAME FROM INFORMATION_SCHEMA.TABLES WHERE TABLE_TYPE = 'BASE TABLE'")

# Fetch all table names
table_names = [table[0] for table in tables.fetchall()]

# Print the list of table names
if table_names == []:
    print('Table not found!')
    create_table_node()
    create_table_edge()
else:
    if not table_names.contains("Edge"):
        create_table_edge()
    if not table_names.contains("Node"):
        crate_table_node()
