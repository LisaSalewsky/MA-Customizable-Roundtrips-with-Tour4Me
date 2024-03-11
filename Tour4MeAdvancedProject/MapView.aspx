﻿<%@ Page Title="" Language="C#" MasterPageFile="~/site.Master" AutoEventWireup="true" CodeBehind="MapView.aspx.cs" Inherits="Tour4MeAdvancedProject.MapView" %>


<asp:Content ID="Content1" ContentPlaceHolderID="head" runat="server">

    <link rel="stylesheet" href="bootstrap/css/bootstrap.min.css" />
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css">
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
        //if (!MyGlobalVariables) {
        //    MyGlobalVariables = {};
        //}
        //MyGlobalVariables.MapUrl = "@Url.Action("Map")";
    </script>

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
    <label for="burgermenucheck" id="menu-toggle-btn">
        <i class =" fas fa-bars" id="btn"></i>
    </label>
    <div class="sidebar">
        <header>Options</header>
        <label for="burgermenucheck" id="menu-toggle-cancel">
          <i class =" fas fa-times" id="cancel"></i>
        </label>
        <form>
            <div class="form-group row first-row"> 
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label"  for="activity">Activity 
                        <span class="tooltip">Pre-selects possibly wanted values based on selected activity</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7  label-no-padding">
                    <select id="activity" name="activity"  class="form-control form-control-sm sidemenu-elements">
                        <option value="select"> Select</option>
                        <option value="running"> Running</option>
                        <option value="cycling"> Cycling</option>
                        <option value="hiking"> Hiking</option>
                    </select>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label"  for="algorithm">Algorithm 
                        <span class="tooltip">Select algorithm to compute tour (can affect tour quality and runtime)</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7  label-no-padding">
                    <select id="algorithm" name="algorithm"  class="form-control form-control-sm sidemenu-elements">
                    </select>
                </div>                
            </div>
            <div class="form-group row label-no-padding">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="length">Length
                        <span class="tooltip">Choose maximum length the generated tour should have (the result will be close to this selected length but never longer)</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7  label-no-padding">
                  <input type="text" class="form-control sidemenu-elements" id="length">
                </div>
                <div class="form-group col-md-1 label-no-padding">
                  km
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="surface">Surface
                        <span class="tooltip">Surface describes properies of the ground. You can select (positive, green/ negative, red) and deselect (transparent)</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7 label-no-padding">
                    <div class="btn-group tagsSurface flex-wrap sidemenu-btn" id="tagButtonsSurface">

                    </div>
                    <br>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="pathType">Path type 
                        <span class="tooltip">Path types describe kinds of paths. They relate to a classification like "highway" or paths only meant for cycling</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7 label-no-padding">
                    <div class="btn-group tagsHighway flex-wrap sidemenu-btn" id="tagButtonsHighway">

                    </div>
                </div>
                               
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="surroundings">Surroundings 
                        <span class="tooltip">The surroundings describe the kind of area you want to visit. After choosing a category, you can select (positive, green/ negative, red) and deselect (transparent)</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7 label-no-padding">
                    <select id="surroundings" name="surroundings"  class="form-control form-control-sm sidemenu-elements" 
                        onselect="changeSurroundingsButtonSelection(this.value)" onchange="changeSurroundingsButtonSelection(this.value)">
                        <option value="Select"> Select</option>
                    </select>
                </div>
                <div class="form-group col-md-4 label-no-padding wrapper">
                </div>
                <div class="form-group col-sm-7 label-no-padding">
                    <div class="btn-group tagsSurroundings flex-wrap sidemenu-btn" id="tagButtonsSurroundings">

                    </div>
                    <br>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="elevation">Elevation
                        <span class="tooltip">Elevation describes the overall maximum elevation the path may have.</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7 label-no-padding">
                  <input type="text" class="form-control sidemenu-elements" id="elevation">
                </div>
                <div class="form-group col-md-1 label-no-padding">
                  %
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="descent">Descent
                        <span class="tooltip">Descent describes the maximum steepness any part of the path may have. This does differentiate between uphill and downhill, descent measuring strictly downhill.</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7 label-no-padding">
                  <input type="text" class="form-control sidemenu-elements" id="descent">
                </div>
                <div class="form-group col-md-1 label-no-padding">
                  %
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="shape">Tour shape
                        <span class="tooltip">The tour shape describes how you would like the final tour to be: as round as possible, one path with a U-turn at the end or a complex path with possibly many turns and crossings</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-7 label-no-padding">
                    <select id="shape" name="shape"  class="form-control form-control-sm sidemenu-elements">
                        <option value="select"> Select</option>
                    </select>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="runningTime">Running time
                        <span class="tooltip">Choose a maximum time (10-300) for the algorithms to run. Small values might lead to no results for some algorithms</span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-6 label-no-padding">
                    <input type="range" class="form-range form-control sidemenu-elements" oninput="changeRangeRunningTime(this, 'runningTime')"
                                   onchange="changeRangeRunningTime(this, 'runningTime')" min="10" max="300" id="runningTime">
                </div>
                <div class="col col-md-1">
                    <label class="form-label out-label" id="runningTimeLabel">30s</label>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="edgeProfit">Edge Profit
                        <span class="tooltip"></span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-6 label-no-padding">
                    <input type="range" class="form-range form-control sidemenu-elements" oninput="changeRanges(this, 'coveredArea')"
                                   onchange="changeRanges(this, 'coveredArea')" min="0" max="100" id="edgeProfit">
                </div>
                <div class="col col-md-1">
                    <label class="form-label out-label" id="edgeProfitLabel">100%</label>
                </div>
            </div>
            <div class="form-group row">
                <div class="form-group col-md-4 label-no-padding wrapper">
                    <label  class="label-no-padding icon-label" for="coveredArea">Covered Area
                        <span class="tooltip"></span>
                        <span><i class="fas fa-info-circle icon-blue" ></i></span>
                    </label> 
                </div>
                <div class="form-group col-sm-6 label-no-padding">
                    <input type="range" class="form-range form-control sidemenu-elements" oninput="changeRanges(this, 'edgeProfit')"
                                   onchange="changeRanges(this, 'edgeProfit')" min="0" max="100" id="coveredArea">
                </div>
                <div class="col col-md-1">
                    <label class="form-label out-label"  id="coveredAreaLabel">100%</label>
                </div>
            </div>
            <div class="form-group row btn-row">
                <div class="col-sm-12" id="btn-row">
                    <asp:Button ID="GetPathButton" class="btn btn-primary" runat="server" Text="Compute Path" onClientClick="return false;"/>
                </div>
            </div>

        </form>
    </div>

    <section id="map">
    </section>
    
    <section id="inputs_bottom">
        <header>Route information</header>
        <form>
             <div id="route_overview" class="form-group row first-row">
                 <details>
                     <summary>Tour 1</summary>
                     <div class="form-group row output-info-box">

                       <div class="form-group column left">
                           <div class="form-group row">
                                <div class="form-group col-md-5">
                                    <label class="routeInfoOutputLabels" for="lengthOut">Overall length
                                    </label>
                                </div>
                                <div class="form-group col-md-6 out-label">
                                     <label id="lengthOut">test</label>
                                </div>
                                <div class="form-group col-md-1">
                                  km
                                </div>
                            </div>
                           <div class="form-group row">
                                <div class="form-group col-md-5">
                                    <label  class="routeInfoOutputLabels" for="elevationUpOut">Elevation uphill
                                    </label>
                                </div>
                                <div class="form-group col-md-6 out-label">
                                     <label id="elevationUpOut"></label>
                                </div>
                                <div class="form-group col-md-1">
                                  %
                                </div>
                            </div>
                           <div class="form-group row">
                                <div class="form-group col-md-5">
                                    <label  class="routeInfoOutputLabels" for="elevationDownOut">Elevation downhill
                                    </label>
                                </div>
                                <div class="form-group col-md-6 out-label">
                                     <label id="elevationDownOut"></label>
                                </div>
                                <div class="form-group col-md-1">
                                  %
                                </div>
                            </div>
                           <div class="form-group row">
                                <div class="form-group col-md-5">
                                    <label  class="routeInfoOutputLabels" for="turnsOut">Number of turns
                                    </label>
                                </div>
                                <div class="form-group col-md-6 out-label">
                                     <label id="turnsOut"></label>
                                </div>
                            </div>
                      </div>
                        <div class="form-group column middle">

                       </div>
                       
                       <div class="form-group column right">
                           <div class="form-group row">
                                <div class="form-group col-md-4">
                                    <label class="routeInfoOutputLabels" for="pathTypesOut">Path types
                                    </label>
                                </div>
                                <div class="form-group col-md-7">
                                     <label id="pathTypesOut">test</label>
                                </div>
                            </div>
                           <div class="form-group row">
                                <div class="form-group col-md-4">
                                    <label  class="routeInfoOutputLabels" for="surfacesOut">Surfaces
                                    </label>
                                </div>
                                <div class="form-group col-md-7 out-label">
                                     <label id="surfacesOut"></label>
                                </div>
                            </div>
                           <div class="form-group row">
                                <div class="form-group col-md-4">
                                    <label  class="routeInfoOutputLabels" for="mainSurroundingsOut">Main surroundings
                                    </label>
                                </div>
                                <div class="form-group col-md-7 out-label">
                                     <label id="mainSurroundingsOut"></label>
                                </div>
                            </div>
                           <div class="form-group row">
                                <div class="form-group col-md-4">
                                    <label  class="routeInfoOutputLabels" for="shapeOut">Shape
                                    </label>
                                </div>
                                <div class="form-group col-md-7 out-label">
                                     <label id="shapeOut"></label>
                                </div>
                            </div>
                            <div class="form-group row out-label">
                                <div class="form-group col-md-11">
                                    <asp:Button ID="ExportTourButton1" class="btn btn-yellow" runat="server" Text="Export Tour" onClientClick="return false;"/>
                                </div>
                            </div>
                      </div>
                    </div> 
                </details>
            </div> 
            
            <div class="form-group row first-row">
                <details>
                    <summary>Tour 2</summary>
                        <div class="form-group column left">
                             <div class="form-group row">
                                  <div class="form-group col-md-4">
                                      <label  class="" for="lengthOut2">Length
                                      </label>
                                  </div>
                                  <div class="form-group col-md-7 out-label">
                                       <label id="lengthOut2">test</label>
                                  </div>
                                  <div class="form-group col-md-1">
                                    km
                                  </div>
                              </div>
                        </div>
                        <div class="form-group column middle">
                        </div>
                        <div class="form-group column right">
                        </div>
               </details>   
            </div> 
        </form>

    </section>

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

    <script>
        //$(document).ready(function () {
        //    $('[data-toggle="tooltip"]').tooltip({
        //        placement: 'bottom'
        //    });
        //});
    </script>

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
                    console.log('Made first ajax call!')

                },
                error: function (xhr, status) {
                    document.getElementById("overlay").style.display = "none";
                    alert("Failed to contact server");
                }
            })

        };

        var max_lat, min_lat, max_lon, min_lon;

        console.log(window.location.search);
        const urlParams = new URLSearchParams(window.location.search);

        map.on('click', onClick);



        function changeSurroundingsButtonSelection(newSelection) {


            i = 0;
            tagsSurroundings.forEach(tag => {

                secondTagButtons = tagButtonsSurroundings;
                const button = document.getElementById("surroundings" + i);

                if (button) {

                    let newClasses = "btn-outline-neutral neutral";
                    button.className = "btn btn-cycle " + newClasses + " sidemenu-elements";
                }

                const toFind = "<label class=\"btn btn-cycle btn-outline-neutral neutral sidemenu-elements\" id=\"surroundings" + i + "\">" + tag + "</label>";

                const startIndex = secondTagButtons.innerHTML.indexOf(toFind);
                var trimmedHTML = secondTagButtons.innerHTML;
                if (startIndex !== -1) {
                    const endIndex = startIndex + toFind.length;
                    trimmedHTML = secondTagButtons.innerHTML.substring(0, startIndex) + secondTagButtons.innerHTML.substring(endIndex);
                }
                secondTagButtons.innerHTML = trimmedHTML;
                i++;
            });

            tagsSurroundings = getSurroundingsByOverallValue(overallTagsSurroundings, newSelection);

            i = 0;
            tagsSurroundings.forEach(tag => {

                secondTagButtons = tagButtonsSurroundings;

                secondTagButtons.innerHTML += "<label class=\"btn btn-cycle btn-outline-neutral neutral sidemenu-elements\" id=\"surroundings" + i + "\">" + tag + "</label>";

                i++;
            });

            $(document).off("click", ".btn-group.tagsSurroundings .btn-cycle").on("click", ".btn-group.tagsSurroundings .btn-cycle", function () {
                if ($(this).hasClass("neutral")) {
                    $(this).removeClass("neutral");
                    $(this).removeClass("btn-outline-neutral");

                    $(this).addClass("desire");
                    $(this).addClass("btn-green");
                } else if ($(this).hasClass("desire")) {
                    $(this).removeClass("desire");
                    $(this).removeClass("btn-green");

                    $(this).addClass("avoid");
                    $(this).addClass("btn-red");
                } else if ($(this).hasClass("avoid")) {
                    $(this).removeClass("avoid");
                    $(this).removeClass("btn-red");

                    $(this).addClass("neutral");
                    $(this).addClass("btn-outline-neutral");
                }
            });

        }

        $.ajax({
            type: 'POST',
            url: "MapView.aspx/RenderGraph",
            contentType: "application/json; charset=utf-8",
            dataType: 'json',
            success: function (result) {
                tagsHighway = result.d.highway;
                tagsSurface = result.d.surface;
                overallTagsSurroundings = result.d.surroundings;

                surroundingsSelect = document.getElementById("surroundings");

                overallTagsSurroundings.forEach(tagCategory => {

                    const pair = tagCategory.Value.split(':');
                    const outerType = pair[0].trim();
                    surroundingsSelect.options[surroundingsSelect.options.length] = new Option(outerType, outerType);

                });



                var surroundingsClass = document.getElementById("surroundings").value;
                tagsSurroundings = getSurroundingsByOverallValue(result.d.surroundings, surroundingsClass);
                algos = result.d.algorithms;
                shape = result.d.shapes;

                marker = L.marker([center_lat, center_lon], { draggable: 'true' }).addTo(map);

                map.on('click', onClick);

                tagButtonsHighway = document.getElementById("tagButtonsHighway");
                tagButtonsSurface = document.getElementById("tagButtonsSurface");
                tagButtonsSurroundings = document.getElementById("tagButtonsSurroundings");

                tagsHighway.forEach(tag => {

                    tagButtons = tagButtonsHighway;

                    const key_val_keys = Object.keys(tag);

                    tagButtons.innerHTML += "<label class=\"btn btn-cycle btn-green desire sidemenu-elements\" id=\"highway" + tag[key_val_keys[0]] + "\">" + tag[key_val_keys[1]] + "</label>"
                });


                tagsSurface.forEach(tag => {
                    
                    tagButtons = tagButtonsSurface;

                    const key_val_keys = Object.keys(tag);

                    tagButtons.innerHTML += "<label class=\"btn btn-cycle btn-outline-neutral neutral sidemenu-elements\" id=\"surface" + tag[key_val_keys[0]] + "\">" + tag[key_val_keys[1]] + "</label>"
                });

                var i = 0;
                tagsSurroundings.forEach(tag => {

                    secondTagButtons = tagButtonsSurroundings;
                    secondTagButtons.innerHTML += "<label class=\"btn btn-cycle btn-outline-neutral neutral sidemenu-elements\" id=\"surroundings" + i + "\">" + tag + "</label>";

                    i++;
                });

                algosSelect = document.getElementById("algorithm");

                algos.forEach(algorithm => {

                    algosSelect.options[algosSelect.options.length] = new Option(algorithm.Value, algorithm.Key);

                });

                
                shapeSelect = document.getElementById("shape");

                algos.forEach(algorithm => {

                    shapeSelect.options[shapeSelect.options.length] = new Option(shape.Value, shape.Key);

                });




                $(".btn-group > .btn-cycle").click(function () {
                    if ($(this).hasClass("neutral")) {
                        $(this).removeClass("neutral");
                        $(this).removeClass("btn-outline-neutral");

                        $(this).addClass("desire");
                        $(this).addClass("btn-green");
                    } else if ($(this).hasClass("desire")) {
                        $(this).removeClass("desire");
                        $(this).removeClass("btn-green");

                        $(this).addClass("avoid");
                        $(this).addClass("btn-red");
                    } else if ($(this).hasClass("avoid")) {
                        $(this).removeClass("desire"); edgeProfit
                        $(this).removeClass("btn-red");

                        $(this).addClass("neutral");
                        $(this).addClass("btn-outline-neutral");
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

        function getSurroundingsByOverallValue(surroundingsArray, surroundingsClass) {

            for (let i = 0; i < surroundingsArray.length; i++) {
                const pair = surroundingsArray[i].Value.split(':');
                const outerType = pair[0].trim();
                const innerTypes = pair[1].trim().split(',');

                // Check if surroundingsClass matches outerType
                if (outerType === surroundingsClass) {
                    // Remove single quotes from inner types and return as array
                    const cleanedInnerTypes = innerTypes.map(type => type.trim().replace(/'/g, ''));
                    return cleanedInnerTypes;
                }
            }
            // If no match found, return an empty array
            return [];
        }

        function changeRanges(current, target) {
            label = document.getElementById(current.id + "Label")
            label.innerHTML = (current.value) + "%"


            document.getElementById(target).value = 100 - current.value;
            // $("#" + target).slider('refresh');

            targetLabel = document.getElementById(target + "Label")
            targetLabel.innerHTML = (100 - current.value) + "%"
        }


        function changeRangeRunningTime(current, target) {
            label = document.getElementById(current.id + "Label")
            label.innerHTML = (current.value) + "s"

            document.getElementById(target).value = current.value;

        }


        $("#<%= GetPathButton.ClientID %>").click(function getPath() {

            var lat = marker.getLatLng()["lat"];
            var lon = marker.getLatLng()["lng"];
            var dis = document.getElementById("length").value * 1000;
            var elevation= document.getElementById("elevation").value ;

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

            var i = 0;
            var surroundingString = "";
            tagsSurroundings.forEach(tag => {
                tag_button = document.getElementById("surroundings" + i);
                if (tag_button.classList.contains("neutral")) {
                    surroundingString += tag + ",n;";
                } else if (tag_button.classList.contains("desire")) {
                    surroundingString += tag + ",d;";
                } else if (tag_button.classList.contains("avoid")) {
                    surroundingString += tag + ",a;";
                }

                i++;
            });

            var surroundingType = document.querySelector('select[name="surroundings"]').value;
            tagsSurroundings = surroundingType + ": " + surroundingString;

            var algorithm = document.querySelector('select[name="algorithm"]').value;

            var runningTime = document.getElementById("runningTime").value;

            var tourShape = document.getElementById("shape").value;

            var edgeProfit = document.getElementById("edgeProfit").value / 100;
            var coveredArea = document.getElementById("coveredArea").value / 100;

            var elevation = document.getElementById("elevation").value;
            var descent = document.getElementById("descent").value;

            function createKeyValuePairArray(pairList) {
                return pairList.map(pair => ((pair[key_val_keys[0]], pair[key_val_keys[1]] )));
            }

            var dataToSend = {
                latIn: lat,
                lonIn: lon,
                distIn: dis,
                algoIn: algorithm,
                elevationIn: elevation,
                descentIn: descent,
                surroundingsIn: tagsSurroundings,
                tagsHIn: createKeyValuePairArray(tagsHighway),
                tagsSIn: createKeyValuePairArray(tagsSurface),
                tourShapeIn: tourShape,
                runningTimeIn: runningTime,
                edgeProfitIn: edgeProfit,
                coveredAreaIn: coveredArea
            };

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

                        var last10Points = path.slice(-10);

                        // Define different colors for markers
                        var colors = ['blue', 'green', 'orange', 'purple', 'pink', 'cyan', 'magenta', 'yellow', 'brown', 'gray'];

                        // Create markers for the last 10 points
                        for (var i = 0; i < last10Points.length; i++) {
                            var color = colors[i % colors.length];
                            L.marker(last10Points[i]).addTo(map);
                        }


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
