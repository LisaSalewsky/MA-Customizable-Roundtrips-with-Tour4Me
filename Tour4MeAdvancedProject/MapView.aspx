<%@ Page Title="" Language="C#" MasterPageFile="~/site.Master" AutoEventWireup="true" CodeBehind="MapView.aspx.cs" Inherits="Tour4MeAdvancedProject.MapView" %>


<asp:Content ID="Content1" ContentPlaceHolderID="head" runat="server">

    <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css" />
     <link rel="stylesheet" href="css/site.css" asp-append-version="true" />
     <%--<link rel="stylesheet" href="~/MyFirstApp.styles.css" asp-append-version="true" />--%>
     <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="" />
     <link rel="stylesheet" href="https://unpkg.com/leaflet-control-geocoder/dist/Control.Geocoder.css"/>

    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="" />
    
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
    <script src="https://unpkg.com/leaflet-sidebar@0.2.0/src/L.Control.Sidebar.js"></script>
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.1.0/jquery.min.js"></script>
    <script src="https://unpkg.com/leaflet-control-geocoder/dist/Control.Geocoder.js"></script>
    <script type="text/javascript">
        if (!MyGlobalVariables) {
            MyGlobalVariables = {};
        }
        //MyGlobalVariables.MapUrl = "@Url.Action("Map")";
    </script>

</asp:Content>



<asp:Content ID="SearchBarPlaceholder" ContentPlaceHolderID="SearchBarPlaceholder" runat="server"> 


</asp:Content>

<asp:Content ID="Content2" ContentPlaceHolderID="ContentPlaceholder" runat="server">



<div id="overlay"
     style="position: fixed; display:none; width: 100%; height: 100%; top:0; left:0; background-color: rgba(0,0,0,0.2); z-index:2;">
    <div id="text"
         style="position: absolute; top: 50%; left: 50%; font-size: 50px; color: white; transform: translate(-50%,-50%); -ms-transform: translate(-50%,-50%);">
        Calculating route
        <div class="loader"></div>
    </div>
</div>


<div id="container">
    <input type="checkbox" id="burgermenucheck"/>
    <label for="burgermenucheck" id="menuToggle">
        <i class =" fas fa-bars" id="btn"></i>
        <i class =" fas fa-times" id="cancel"></i>
    </label>
    <div class="sidebar">
        <header>Options</header>
        <form>
            <div class="form-group row"> 
                <div class="form-group col-md-4">
                    <label for="activity">Activity</label> 
                </div>
                <div class="form-group col-sm-7">
                    <select id="activity" name="activity"  class="form-control form-control-sm">
                        <option value="select"> Select</option>
                        <option value="running"> Running</option>
                        <option value="cycling"> Cycling</option>
                        <option value="hiking"> Hiking</option>
                    </select>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4">
                    <label for="algorithm">Algorithm</label> 
                </div>
                <div class="form-group col-sm-7">
                    <select id="algorithm" name="algorithm"  class="form-control form-control-sm">
                        <option value="greedy"> Greedy</option>
                        <option value="mincost"> minCost</option>
                        <option value="ils"> ILS</option>
                        <option value="ant"> Ant</option>
                        <option value="genetic"> Genetic</option>
                    </select>
                </div>                
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4">
                  <label for="length">Length</label>
                </div>
                <div class="form-group col-sm-7">
                  <input type="text" class="form-control" id="length"> km
                </div>
            </div>
            <div class="form-group row">
                
            </div>
            <div class="form-group row">
                
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4">
                    <label for="surroundings">Surroundings</label> 
                </div>
                <div class="form-group col-sm-7">
                    <select id="surroundings" name="surroundings"  class="form-control form-control-sm">
                        <option value="select"> Select</option>
                        <option value="park"> Park</option>
                        <option value="forest"> Forest</option>
                        <option value="mountains"> Mountains</option>
                        <option value="beach"> Beach</option>
                        <option value="city"> City</option>
                    </select>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4">
                  <label for="elevation">Elevation</label>
                </div>
                <div class="form-group col-sm-7">
                  <input type="text" class="form-control" id="elevation"> %
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4">
                    <label for="shape">Tour shape</label> 
                </div>
                <div class="form-group col-sm-7">
                    <select id="shape" name="shape"  class="form-control form-control-sm">
                        <option value="select"> Select</option>
                        <option value="round"> Round</option>
                        <option value="uturn"> U-Turn</option>
                        <option value="complex"> Complex</option>
                    </select>
                </div>
            </div>
            <div class="form-group row">
                <div class="col-sm-10">
                  <button type="submit" class="btn btn-primary">Sign in</button>
                </div>
            </div>

        </form>
<%--        <ul id="menu">
            <li><label for="activity">Activity</label> 
                <select id="activity" name="activity">
                    <option value="select"> Select</option>
                    <option value="running"> Running</option>
                    <option value="cycling"> Cycling</option>
                    <option value="hiking"> Hiking</option>
                </select></li>
            <li>About</li>
            <li>Info test name long longer longest</li>
            <li>Contact</li>
            <a href="https://erikterwan.com/" target="_blank"><li>Show me more</li></a>
        </ul>--%>
    </div>

    <section id="map">
    </section>

    

    <section id="inputs_bottom">

        <div class="input-group mb-3">
        <form>
            <div class="form-group row"> 
                <div class="form-group col-md-4">
                    <label for="activitys">Activity</label> 
                </div>
                <div class="form-group col-sm-7">
                    <select id="activitys" name="activitys"  class="form-control form-control-sm">
                        <option value="select"> Select</option>
                        <option value="running"> Running</option>
                        <option value="cycling"> Cycling</option>
                        <option value="hiking"> Hiking</option>
                    </select>
                </div>
            </div>
            </form>

            <input type="text" id="distance" class="form-control" value="50" aria-label="Target Distance (m)">
            <div class="input-group-append">
                <button class="btn btn-secondary" disabled type="button">km</button>
                    <%--<button type="button" class="btn btn-primary" onclick="getPath()">Compute Path</button>--%>
                <asp:Button ID="GetPathButton" class="btn btn-primary" runat="server" Text="Compute Path" onClientClick="return false;"/>
            </div>
            <div>

            </div>
        </div>


        <div class="btn-group" id="algoRadio">
        </div>

        <button type="button" class="btn btn-warning" data-bs-toggle="modal" style="position:absolute; right:20px"
                data-bs-target="#settingsModal">
            Open settings
        </button>

    </section>

    <!-- Modal -->
    <div class="modal fade" id="settingsModal" tabindex="-1" aria-labelledby="settingsModalLabel" aria-hidden="false">
        <div class="modal-dialog modal-lg ">
            <div class="modal-content">
                <div class="modal-header">
                    <h3 class="modal-title" id="settingsModalLabel">Settings</h3>
                    <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
                </div>
                <div class="modal-body">
                    <h5>Underlying map:</h5>
                    <div class="btn-group" style="display: block; ">
                        <input type="radio" class="btn-check" name="mapType" value="b" id="backbone"
                               autocomplete="off" />
                        <label class="btn btn-outline-success" for="backbone">Backbone</label>
                        <input type="radio" class="btn-check" name="mapType" value="o" id="osm" autocomplete="off"
                               checked />
                        <label class="btn btn-outline-success" for="osm">OSM</label>
                    </div>
                    <br>
                    <h5>Type of roads:</h5>
                    <div class="btn-group tagsHighway pre-scrollable"
                         style="display: block; overflow-x: auto; white-space: nowrap;" id="tagButtonsHighway"></div>
                    <br>
                    <h5>Type of surfaces:</h5>
                    <div class="btn-group mr-2 tagsSurface pre-scrollable"
                         style="display: block; overflow-x: auto; white-space: nowrap;" id="tagButtonsSurface"></div>
                    <br>
                    <h5>Running time (max 300s):</h5>
                    <div class="input-group">
                        <input type="text" class="form-control" value="30" aria-label="Running Time" id="runningTime">
                        <span class="input-group-text">seconds</span>
                    </div>
                    <br>
                    <h5>Quality Distribution:</h5>
                    <div class="row">
                        <div class="col col-md-2">
                            <label class="form-label">Edge Profit</label>
                        </div>
                        <div class="col col-md-9">
                            <input type="range" class="form-range" oninput="changeRanges(this, 'coveredArea')"
                                   onchange="changeRanges(this, 'coveredArea')" min="0" max="100" id="edgeProfit">
                        </div>
                        <div class="col col-md-1">
                            <label class="form-label" style="float: right;" id="edgeProfitLabel">100%</label>
                        </div>
                    </div>
                    <div class="row">
                        <div class="col col-md-2">
                            <label class="form-label">Covered Area</label>
                        </div>
                        <div class="col col-md-9">
                            <input type="range" class="form-range" oninput="changeRanges(this, 'edgeProfit')"
                                   onchange="changeRanges(this, 'edgeProfit')" min="0" max="100" id="coveredArea">
                        </div>
                        <div class="col col-md-1">
                            <label class="form-label" style="float: right;" id="coveredAreaLabel">100%</label>
                        </div>
                    </div>
                </div>
                <div class="modal-footer">
                    <button type="button" class="btn btn-primary" data-bs-dismiss="modal">Close</button>
                </div>
            </div>
        </div>
    </div>

    

    <div class="modal fade" id="infoModal" tabindex="-1" aria-labelledby="infoModalLabel" aria-hidden="false">
        <div class="modal-dialog modal-lg ">
            <div class="modal-content">
                <div class="modal-header">
                    <h3 class="modal-title" id="infoModalLabel">Route Information</h3>
                    <button type="button" class="btn-close" data-bs-dismiss="modal" aria-label="Close"></button>
                </div>
                <div class="modal-body">
                    <!-- <h5></h5> -->
                    <div id="metadata-body">
                        <p>No route exists yet!</p>
                    </div>
                </div>
                <div class="modal-footer">
                    <button type="button" class="btn btn-primary" data-bs-dismiss="modal">Close</button>
                </div>
            </div>
        </div>
    </div>
</div>
    
    <script>
        const checkbox = document.getElementById('check');
        const btn = document.getElementById('btn');
        const cancel = document.getElementById('cancel');

        btn.addEventListener('click', function () {
            checkbox.checked = !checkbox.checked; // Toggle the checkbox state
        });

        cancel.addEventListener('click', function () {
            checkbox.checked = false; // Set checkbox state to unchecked
        });
    </script>


    <script>
        var map = L.map('map').setView([0, 0], 13);

        L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        }).addTo(map);
        L.Control.geocoder({
            defaultMarkGeocode: false
        })
            .on('markgeocode', function (e) {
                if (marker) {
                    map.removeLayer(marker);
                    marker.setLatLng(new L.LatLng(e.geocode.center.lat, e.geocode.center.lng), { draggable: 'true' });
                    marker.addTo(map);
                    map.panTo(new L.LatLng(e.geocode.center.lat, e.geocode.center.lng))
                    centered = true;
                } else {
                    marker = L.marker(new L.LatLng(e.geocode.center.lat, e.geocode.center.lng), { draggable: 'true' }).addTo(map);
                    map.panTo(new L.LatLng(e.geocode.center.lat, e.geocode.center.lng))
                    centered = true;
                }
            }).addTo(map);


        var marker;
        var zoomed = true;
        var outer_box;
        var inner_box;


        var tags;
        var algos;

        const lat_sea = 47.45;
        const lon_sea = -122.4;
        const siz_sea = 2;

        const lat_dor = 51.3648;
        const lon_dor = 7.2185;

        lat_gran = 0.5 / 4
        lat_pad = 0.5 / 6;
        abs_min_lat = lat_dor;
        abs_max_lat = abs_min_lat + lat_gran;

        lon_gran = 0.75 / 4
        lon_pad = 0.75 / 6;
        abs_min_lon = lon_dor;
        abs_max_lon = abs_min_lon + lon_gran;
        const colors = ["red", "green", "blue", "maroon", "purple", "lime", "navy"]

        var selectedMap = "dor"

        var route_counter = 0;

        var stored_routes = [];

        center_lat = 51.489808;
        center_lon = 7.406319;

        var centered = false;


        setTimeout(function () { map.invalidateSize(true) }, .1);

        navigator.geolocation.watchPosition(success, error);


        function success(pos) {
            console.log("success");
            ////////console.log($("#<%= GetPathButton.ClientID %>");

            var lat = pos.coords.latitude;
            var lng = pos.coords.longitude;
            const accuracy = pos.coords.accuracy;
            zoomed = false;

            if (!centered) {
                lat = center_lat;
                lng = center_lon;
                map.setView([lat, lng]);
                centered = true;
            }


            if (marker) {
                map.removeLayer(marker);
            } else {
                marker = L.marker([center_lat, center_lon], { draggable: 'true' }).addTo(map);
            }

            // when marker is dragged
            marker.on('dragend', function (event) {
                var marker = event.target;
                var position = marker.getLatLng();
                marker.setLatLng(new L.LatLng(position.lat, position.lng), { draggable: 'true' });
                map.panTo(new L.LatLng(position.lat, position.lng))
                centered = true;
            });
            marker.addTo(map)


            // Set map focus to current user position

        }

        function error(err) {

            if (err.code === 1) {
                alert("Please allow geolocation access");
            } else {
                alert("Cannot get current location");
            }

        }


        var onClick = function (e) {

            if (marker) {
                map.removeLayer(marker);
            }
            marker.setLatLng(e.latlng);
            marker.addTo(map);
            center_lat = e.latlng.lat;
            center_lon = e.latlng.lng;
            map.panTo(new L.LatLng(center_lat, center_lon))
            centered = true;

            var marker_lat = marker.getLatLng().lat;
            var marker_lon = marker.getLatLng().lng;

            centered = false;

            $.ajax({
                type:'POST',
                url: "MapView.aspx/setlatlon",
                data: JSON.stringify({ latIn: marker_lat, lonIn: marker_lon }),
                contentType: 'application/json; charset=utf-8',
                dataType: 'json',
                success: function (result) {
                    // center_lat = result["center_lat"];
                    // center_lon = result["center_lon"];
                    console.log('Made first ajax call!')

                },
                error: function (xhr, status) {
                    document.getElementById("overlay").style.display = "none";
                    alert("Failed to contact server");
                }
            })

        };


        // $(document).ready(function () {

        // var center_lat, center_lon;
        var max_lat, min_lat, max_lon, min_lon;

        console.log(window.location.search);
        const urlParams = new URLSearchParams(window.location.search);

        map.on('click', onClick);


        //but = document.getElementById("switcherDor");
        //if (urlParams.has("map")) {
        //    if (urlParams.get("map") == "sea") {
        //        but = document.getElementById("switcherSea");
        //        selectedMap = "sea";

        //        abs_min_lat = lat_sea;
        //        abs_max_lat = abs_min_lat + lat_gran;
        //        lat_gran = 0.5 / 2
        //        lat_pad = 0.5 / 4

        //        abs_min_lon = lon_sea;
        //        abs_max_lon = abs_min_lon + lon_gran;

        //        lon_gran = 0.75 / 2
        //        lon_pad = 0.75 / 4;

        //        center_lat = (abs_max_lat + abs_min_lat) / 2;
        //        center_lon = (abs_max_lon + abs_min_lon) / 2;
        //    }
        //}

        //but.setAttribute("aria-disabled", "true");
        //but.classList.add("disabled");


        $.ajax({
            type: 'POST',
            url: "MapView.aspx/RenderGraph",
            contentType: "application/json; charset=utf-8",
            dataType: 'json',
            success: function (result) {
                // center_lat = result["center_lat"];
                // center_lon = result["center_lon"];

                tagsHighway = result.d.highway;
                tagsSurface = result.d.surface;
                algos = result.d.algorithms;

                //map = L.map('map').setView([center_lat, center_lon], 14);

                //var tiles = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=pk.eyJ1IjoibWFwYm94IiwiYSI6ImNpejY4NXVycTA2emYycXBndHRqcmZ3N3gifQ.rJcFIG214AriISLbB6B5aw', {
                //    maxZoom: 25,
                //    attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, ' +
                //        'Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
                //    id: 'mapbox/streets-v11',
                //    tileSize: 512,
                //    zoomOffset: -1
                //}).addTo(map);

                // var polyline = L.polyline(path, {color: 'red'}).addTo(map);

                // zoom the map to the polyline

                marker = L.marker([center_lat, center_lon], { draggable: 'true' }).addTo(map);

                //outer_box = L.polygon(
                //    [[[900, -1800],
                //    [900, 1800],
                //    [-900, 1800],
                //    [-900, -1800]], // outer ring
                //    [[abs_min_lat - lat_pad, abs_min_lon - lon_pad],
                //    [abs_min_lat - lat_pad, abs_max_lon + lon_pad],
                //    [abs_max_lat + lat_pad, abs_max_lon + lon_pad],
                //    [abs_max_lat + lat_pad, abs_min_lon - lon_pad]] // actual cutout polygon
                //    ], { interactive: true, color: 'black', weight: "0", fillOpacity: 0.08 }).addTo(map);

                //inner_box = L.polygon(
                //    [[abs_min_lat - lat_pad, abs_min_lon - lon_pad],
                //    [abs_min_lat - lat_pad, abs_max_lon + lon_pad],
                //    [abs_max_lat + lat_pad, abs_max_lon + lon_pad],
                //    [abs_max_lat + lat_pad, abs_min_lon - lon_pad]
                //    ], { weight: "0", fillOpacity: 0 }).addTo(map);


                //outer_box.on('click', function (e) {
                //    outer_box.setStyle({ color: "red", fillOpacity: 0.15 });
                //});

                //inner_box.on('click', function (e) {
                //    outer_box.setStyle({ color: "black", fillOpacity: 0.08 });
                //});

                map.on('click', onClick);

                tagButtonsHighway = document.getElementById("tagButtonsHighway");
                tagButtonsSurface = document.getElementById("tagButtonsSurface");

                tagsHighway.forEach(tag => {

                    tagButtons = tagButtonsHighway;

                    const key_val_keys = Object.keys(tag);

                    tagButtons.innerHTML += "<label class=\"btn btn-cycle btn-success desire\" id=\"highway" + tag[key_val_keys[0]] + "\">" + tag[key_val_keys[1]] + "</label>"
                });

                tagsSurface.forEach(tag => {
                    
                    tagButtons = tagButtonsSurface;

                    const key_val_keys = Object.keys(tag);

                    tagButtons.innerHTML += "<label class=\"btn btn-cycle btn-outline-secondary neutral\" id=\"surface" + tag[key_val_keys[0]] + "\">" + tag[key_val_keys[1]] + "</label>"
                });


                algosRadio = document.getElementById("algoRadio");

                algos.forEach(algorithm => {

                //    const key_val_keys = Object.keys(algorithm);

                //    algosRadio.innerHTML += "<input type=\"radio\" class=\"btn-check\" name=\"algorithm\" value=" + algorithm[key_val_keys[0]] + " id=\"" + algorithm[key_val_keys[1]] + "\" autocomplete=\"off\" " + (algorithm[key_val_keys[0]] == 1 ? "checked" : "") + "/>";
                //    algosRadio.innerHTML += "<label class=\"btn btn-primary\" for=\"" + algorithm[key_val_keys[0]] + "\">" + algorithm[key_val_keys[1]] + "</label>";

                //    console.log(algorithm[key_val_keys[1]] + " Button created")
                //});

                    const radioInput = document.createElement("input");
                    radioInput.type = "radio";
                    radioInput.className = "btn-check";
                    radioInput.name = "algorithm";
                    radioInput.value = algorithm.Key;
                    radioInput.id = "algo" + algorithm.Key;
                    radioInput.autocomplete = "off";
                    radioInput.checked = algorithm.Key === 1; // Set checked based on some condition

                    const label = document.createElement("label");
                    label.className = "btn btn-primary";
                    label.htmlFor = "algo" + algorithm.Key;
                    label.textContent = algorithm.Value;

                    algosRadio.appendChild(radioInput);
                    algosRadio.appendChild(label);

                    // Add event listener to each radio button
                    radioInput.addEventListener("change", function () {
                        console.log(algorithm.Value + " Button clicked");
                        // Add your logic here for handling the change event
                    });
                });


                $(".btn-group > .btn-cycle").click(function () {
                    if ($(this).hasClass("neutral")) {
                        $(this).removeClass("neutral");
                        $(this).removeClass("btn-outline-secondary");

                        $(this).addClass("desire");
                        $(this).addClass("btn-success");
                    } else if ($(this).hasClass("desire")) {
                        $(this).removeClass("desire");
                        $(this).removeClass("btn-success");

                        $(this).addClass("avoid");
                        $(this).addClass("btn-danger");
                    } else if ($(this).hasClass("avoid")) {
                        $(this).removeClass("desire"); edgeProfit
                        $(this).removeClass("btn-danger");

                        $(this).addClass("neutral");
                        $(this).addClass("btn-outline-secondary");
                    }
                });

                changeRanges(document.getElementById("edgeProfit"), "coveredArea")
                // getBackbone();

            },
            error: function (xhr, status) {
                document.getElementById("overlay").style.display = "none";
                alert("Failed to contact server");
            }
        })


        // });


        function changeRanges(current, target) {
            label = document.getElementById(current.id + "Label")
            label.innerHTML = (current.value) + "%"


            document.getElementById(target).value = 100 - current.value;
            // $("#" + target).slider('refresh');

            targetLabel = document.getElementById(target + "Label")
            targetLabel.innerHTML = (100 - current.value) + "%"
        }

        $("#<%= GetPathButton.ClientID %>").click(function getPath() {

            var lat = marker.getLatLng()["lat"];
            var lon = marker.getLatLng()["lng"];
            var dis = document.getElementById("distance").value * 1000;

            $.ajax({
                type: 'POST',
                url: "MapView.aspx/GetPath",
                contentType: "application/json; charset=utf-8",
                data: JSON.stringify({
                    latIn: lat,
                    lonIn: lon
                }),
                dataType: 'json',
                success: function (result) {
                    console.log("get path success")
                    
                    max_lat = result["max_lat"];
                    min_lat = result["min_lat"];
                    max_lon = result["max_lon"];
                    min_lon = result["min_lon"];

                    //if (inner_box) {
                    //    inner_box.remove(map);
                    //}

                    //inner_box = l.polygon(
                    //    [
                    //        [
                    //            [abs_min_lat - lat_pad, abs_min_lon - lon_pad],
                    //            [abs_min_lat - lat_pad, abs_max_lon + lon_pad],
                    //            [abs_max_lat + lat_pad, abs_max_lon + lon_pad],
                    //            [abs_max_lat + lat_pad, abs_min_lon - lon_pad]
                    //        ],
                    //        [
                    //            [min_lat, min_lon],
                    //            [min_lat, max_lon],
                    //            [max_lat, max_lon],
                    //            [max_lat, min_lon]
                    //        ]
                    //    ],
                    //    { interactive: false, color: 'yellow' })
                    //    .addto(map);
                },
                error: function (xhr, status) {
                    console.log("get path error")
                    document.getElementById("overlay").style.display = "none";
                    alert("Failed to connect to server!");
                }
            })


            if (isNaN(dis)) {
                alert("Must input numbers");
                return false;
            }

            tag_str = "&tags="

            const key_val_keys = ["Key", "Value"];

            tagsHighway.forEach(tag => {
                tag_button = document.getElementById("highway" + tag[key_val_keys[0]]);
                if (tag_button.classList.contains("neutral")) {
                    tag[key_val_keys[1]] += ",n";
                } else if (tag_button.classList.contains("desire")) {
                    tag[key_val_keys[1]] += ",d";
                } else if (tag_button.classList.contains("avoid")) {
                    tag[key_val_keys[1]] += ",a";
                }
            });

            tagsSurface.forEach(tag => {
                tag_button = document.getElementById("surface" + tag[key_val_keys[0]]);
                if (tag_button.classList.contains("neutral")) {
                    tag[key_val_keys[1]] += ",n";
                } else if (tag_button.classList.contains("desire")) {
                    tag[key_val_keys[1]] += ",d";
                } else if (tag_button.classList.contains("avoid")) {
                    tag[key_val_keys[1]] += ",a";
                }
            });


            var algorithm = document.querySelector('input[name="algorithm"]:checked').value;

            var mapType = document.querySelector('input[name="mapType"]:checked').value;
            console.log(mapType);


            var runningTime = document.getElementById("runningTime").value;

            var edgeProfit = document.getElementById("edgeProfit").value / 100;
            var coveredArea = document.getElementById("coveredArea").value / 100;

            document.getElementById("overlay").style.display = "block";

            function createKeyValuePairArray(pairList) {
                return pairList.map(pair => ((pair[key_val_keys[0]], pair[key_val_keys[1]] )));
            }

            var dataToSend = {
                latIn: lat,
                lonIn: lon,
                distIn: dis,
                algoIn: algorithm,
                tagsHIn: createKeyValuePairArray(tagsHighway),
                tagsSIn: createKeyValuePairArray(tagsSurface),
                runningTimeIn: runningTime,
                edgeProfitIn: edgeProfit,
                coveredAreaIn: coveredArea
            };

            const testKVPair = createKeyValuePairArray(tagsHighway);
            console.log(testKVPair.key);
            console.log(testKVPair.value);
            console.log(createKeyValuePairArray(tagsSurface)); 

            console.log(tagsHighway.map(pair => ((pair[key_val_keys[0]], pair[key_val_keys[1]] ))));
            console.log(tagsSurface.map(pair => ({ Key: pair[key_val_keys[0]], Value: pair[key_val_keys[1]] }))); 


            // Custom serialization for KeyValuePair
            dataToSend = JSON.parse(JSON.stringify(dataToSend, (key, value) => {
                if (value instanceof Object && value.hasOwnProperty('key') && value.hasOwnProperty('value')) {
                    return { key: value.key, value: value.value };
                }
                return value;
            }));


            console.log(JSON.stringify(dataToSend));

            $.ajax({
                type: 'POST',
                url: "MapView.aspx/Tour",
                contentType: "application/json; charset=utf-8",
                data: JSON.stringify(dataToSend),
                dataType: 'json',
                success: function (result) {
                    if ("success" in result.d) {
                        console.log("Tour success" + result.d.success);
                        var path = JSON.parse(result.d.path);

                        var line = L.polyline(path, { color: colors[route_counter % colors.length], weight: 5 });
                        line = line.addTo(map);
                        
                        document.getElementById("overlay").style.display = "none";
                        map.fitBounds(line.getBounds());

                        var metadata = JSON.parse(result.d.meta);

                        m_body = document.getElementById("metadata-body");

                        m_body.innerHTML = ""
                        metadata.forEach(element => {
                            m_body.innerHTML += "<p>" + element + "</p>\n";
                        });

                        stored_routes.push({ 'route_id': route_counter, 'line': line, 'path': path, 'metadata': metadata })

                        html_routes = document.getElementById("route_overview");

                        html_routes.innerHTML += "     <div class=\"input-group mb-3\" id=\"d" + route_counter + "\">"
                            + "        <div class=\"input-group-prepend\">"
                            + "            <button class=\"btn btn-warning\" route_id=\"" + route_counter + "\" onclick=\"toggleRoute(this)\" data-toggle=\"tooltip\" data-placement=\"bottom\""
                            + "                title=\"Toggle Selected Route\">T</button>"
                            + "            <button class=\"btn btn-secondary\" route_id=\"" + route_counter + "\" data-bs-toggle=\"modal\" data-bs-target=\"#infoModal\" onclick=\"showInfo(this)\" data-toggle=\"tooltip\" data-placement=\"bottom\""
                            + "                title=\"Display Route Information\">I</button>"
                            + "        </div>"
                            + ""
                            + "        <input type=\"text\" class=\"form-control\" id=\"r" + route_counter + "\" style=\"color:" + colors[route_counter % colors.length] + "\" value=\"Route " + route_counter + "\">"
                            + ""
                            + "        <div class=\"input-group-append\">"
                            + "            <button class=\"btn btn-success\" route_id=\"" + route_counter + "\" onclick=\"toGPX(this)\" data-toggle=\"tooltip\" data-placement=\"bottom\""
                            + "                title=\"Export Route to GPX\">E</button>"
                            + "            <button class=\"btn btn-danger\" route_id=\"" + route_counter + "\" onclick=\"deleteRoute(this)\"  data-toggle=\"tooltip\" data-placement=\"bottom\""
                            + "                title=\"Delete Route\">D</button>"
                            + "        </div>"
                            + "    </div>"

                        $('[data-toggle="tooltip"]').tooltip({
                            trigger: 'hover'
                        })

                        $('[data-toggle="tooltip"]').on('click', function () {
                            $(this).tooltip('hide')
                        })

                        route_counter += 1;

                    } else if ("error" in result.d) {
                        console.log(result.d.error)
                        document.getElementById("overlay").style.display = "none";
                        alert("Failed to calculate a route!");
                    }


                },
                error: function (xhr, status) {
                    document.getElementById("overlay").style.display = "none";
                    console.log("Tour error");
                    if (xhr.status == 504) {
                        alert("Computation did not find a solution with the given time bound, please increase the running time (settings) and try again.");
                    } else if (xhr.status == 503) {
                        alert("There are to many requests to handle yours right now, please try again later!");
                    } else {
                        alert("Failed to calculate a route!");
                    }
                }
            })

        });

        function findRoute(route_id) {
            var route_obj;
            for (let index = 0; index < stored_routes.length; index++) {
                const element = stored_routes[index];

                if (element["route_id"] == route_id) {
                    return element;
                }
            }
        }

    </script>


</asp:Content>
