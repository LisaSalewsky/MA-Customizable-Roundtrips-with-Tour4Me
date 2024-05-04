# MA-Customizable-Roundtrips-with-Tour4Me
## Meta-heuristic Approaches for Personalized Running and Cycling Routes

This Repo contains all of the Code and Text of the master thesis.\
The Code is be mostly C#, the Text is written using LaTeX and uploaded as PDF as well.

### Prerequisites
For compiling C# code:
 - Visual Studio
   - .NetCore Package (current Target Framework: .NET 4.7.2)

For saving the graph data:
  - Download an install Microsoft SQL Server (https://www.microsoft.com/de-de/sql-server/sql-server-downloads --> select the free express version)
    - After installing, a configuration screen pops up
      - Here, the instance name should be **SQLEXPRESS**
      - Copy the Connection string and save is somewhere for future use  
  - Download and intall Microsoft SQL Server Management Studio (https://learn.microsoft.com/en-us/sql/ssms/download-sql-server-management-studio-ssms?view=sql-server-ver16 --> scroll to **Download SSMS** and click the first link)
    - To connect, "server name" needs to be set and match the pattern "your PC name\ you sql instance name" so, for example "DESKTOP123\SQLEXPRESS" or "localhost\SQLEXPRESS" 
  - Create new Database (Preconfigured: GridDataDB) --> if you want to rename it: change the connection string in C# code (Web.config) and in the python script (getGraph.py)

For getting graph data and their elevation
 - Install Docker
 - go to [Open Elevation Git Repo](https://github.com/Jorl17/open-elevation/blob/master/docs/host-your-own.md) and fetch elevation data
   ``` 
      git clone http://github.com/Jorl17/open-elevation
      cd open-elevation 
   ```
   - On Windows Powershell: 
   ```
      mkdir data # Create the target folder for the dataset
      docker run -t -i -v ${pwd}/data:/code/data openelevation/open-elevation /code/create-dataset.sh
   ```
  - move the imported .tif files into the data folder where the respective zip folders are located
  - To run the server
  ```
    docker run -t -i -v ${pwd}/data:/code/data -p 80:8080 openelevation/open-elevation
  ```
  - or with SSL
  ```
    docker run -t -i -v ${pwd}/data:/code/data -v $(pwd)/certs:/code/certs -p 443:8080 openelevation/open-elevation
  ```
- execute getGraph.py to
  - create all needed Tables
  - generate a graph of the place you want to use
  - query all elevation data from the local server of open elevation run in docker 
  - save the data into the database

### How to build
 - initially run getGraph.py with the location you want to query
   - for this, first start Docker
   - then run the container used for elevation
   - finally, execute the script
 - after importing the data, run ???
 - go to localhost:443??


### Implemented algorithms
- Ant Colony
- Genetic programming
- (Greedy)
- (minCost)
- (ILS)

### Option parameters
- path type
- surface type
- elevation
- tour shape
- surroundings
- length
- algorithm
- (activity --> only prefills fields)