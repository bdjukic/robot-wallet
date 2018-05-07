pragma solidity ^0.4.8;

contract GpsManager {
    string public currentLocation;
    
    function setCurrentLocation(string tempLocation) public {
        currentLocation = tempLocation;
    }
}
