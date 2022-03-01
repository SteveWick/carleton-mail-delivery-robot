from gql import gql, Client
from gql.transport.aiohttp import AIOHTTPTransport
import json

DEBUG=0

database = AIOHTTPTransport(
    url="https://graphql.us.fauna.com/graphql",
    headers={'authorization': 'Bearer fnAEcsZAjRAAQFZKEY2ld2Lf-oxxylhdza1i8r9j',
    'Content-Type': 'application/json'})

def executeAPI(gqlString):
    client = Client(transport=database, fetch_schema_from_transport=True)

    result = client.execute(gqlString)
    if (DEBUG):
        print(result)

    return result


# ************** Requests **************

def addRequest(timestamp, source, destination, sender, recipient, state):
    gqlString = f"""
        mutation AddRequest {{
            createRequest(data: {{
                timestamp: "{timestamp}"
                source: "{source}"
                destination: "{destination}"
                sender: "{sender}"
                recipient: "{recipient}"
                state: "{state}"
            }}) {{
                _id
            }}
        }}
    """
    return gql(gqlString)

def getRequests():
    return gql(
    """
        query GetRequests {
            allRequests {
                data {
                    _id,
                    robot,
                    timestamp,
                    source,
                    destination,
                    sender,
                    recipient,
                    state
                }
            }
        }
    """
    )

def updateRequest(id, robot, timestamp, source, destination, sender, recipient, state):
    gqlString = f"""
        mutation UpdateRequest {{
            updateRequest(id: {id}, data: {{
                robot: {robot}
                timestamp: "{timestamp}"
                source: "{source}"
                destination: "{destination}"
                sender: "{sender}"
                recipient: "{recipient}"
                state: "{state}"
            }}) {{
                _id,
                robot,
                timestamp,
                source,
                destination,
                sender,
                recipient,
                state
            }}
        }}
    """
    return gql(gqlString)

def deleteRequest(id):
    gqlString = f"""
    mutation DeleteRequest {{
        deleteRequest(id: {id}) {{
            _id
        }}
    }}
    """
    return gql(gqlString)

def getOpenRequests():
    return gql(
    """
        query FindOpenRequests {
        requestsByState(state:"pending") {
                data {
                    _id,
                    robot,
                    timestamp,
                    source,
                    destination,
                    sender,
                    recipient,
                    state
                }
            }
        }
    """
    )

def claimRequest(id, robot):
    gqlString = f"""
        mutation UpdateRequest {{
            updateRequest(id: {id}, data: {{
                robot: {robot}
                state: "InProgress"
            }}) {{
                _id,
                robot,
                timestamp,
                source,
                destination,
                sender,
                recipient,
                state
            }}
        }}
    """
    return gql(gqlString)

def completeRequest(id):
    gqlString = f"""
        mutation UpdateRequest {{
            updateRequest(id: {id}, data: {{
                state: "Completed"
            }}) {{
                _id,
                robot,
                timestamp,
                source,
                destination,
                sender,
                recipient,
                state
            }}
        }}
    """
    return gql(gqlString)


# ************** Robots **************

def addRobot(status):
    gqlString = f"""
        mutation AddRobot {{
            createRobot(data: {{
                status: "{status}"
            }}) {{
                _id
            }}
        }}
    """
    return gql(gqlString)

def getRobots():
    return gql(
    """
        query GetRobots {
            allRobots {
                data {
                    _id,
                    location,
                    status
                }
            }
        }
    """
    )

def updateRobot(id, location, status):
    gqlString = f"""
        mutation UpdateRobot {{
            updateRobot(id: {id}, data: {{
                location: "{location}"
                status: "{status}"
            }}) {{
                _id,
                location,
                status
            }}
        }}
    """
    return gql(gqlString)

def deleteRobot(id):
    gqlString = f"""
    mutation DeleteRobot {{
        deleteRobot(id: {id}) {{
            _id
        }}
    }}
    """
    return gql(gqlString)

# ************** Junctions **************

def addJunction():
    gqlString = f"""
        mutation AddJunction {{
            createJunction(data: {{
            }}) {{
                _id
            }}
        }}
    """
    return gql(gqlString)

def getJunctions():
    return gql(
    """
        query GetJunctions {
            allJunctions {
                data {
                    _id,
                    beacons
                }
            }
        }
    """
    )

def updateJunction(id, beacons):
    gqlString = f"""
        mutation UpdateJunction {{
            updateJunction(id: {id}, data: {{
                beacons: {beacons}
            }}) {{
                _id,
                beacons
            }}
        }}
    """
    return gql(gqlString)

def deleteJunction(id):
    gqlString = f"""
    mutation DeleteJunction {{
        deleteJunction(id: {id}) {{
            _id
        }}
    }}
    """
    return gql(gqlString)

# ************** Beacons **************

def addBeacon(status):
    gqlString = f"""
        mutation AddBeacon {{
            createBeacon(data: {{
                status: "{status}"
            }}) {{
                _id
            }}
        }}
    """
    return gql(gqlString)

def getBeacons():
    return gql(
    """
        query GetBeacons {
            allBeacons {
                data {
                    _id,
                    junction,
                    status
                }
            }
        }
    """
    )

def updateBeacon(id, junction, status):
    gqlString = f"""
        mutation UpdateBeacon {{
            updateBeacon(id: {id}, data: {{
                junction: {junction}
                status: "{status}"
            }}) {{
                _id,
                junction,
                status
            }}
        }}
    """
    return gql(gqlString)

def deleteBeacon(id):
    gqlString = f"""
    mutation DeleteBeacon {{
        deleteBeacon(id: {id}) {{
            _id
        }}
    }}
    """
    return gql(gqlString)

def updateBeaconStatus(id, status):
    gqlString = f"""
        mutation UpdateBeacon {{
            updateBeacon(id: {id}, data: {{
                status: "{status}"
            }}) {{
                _id,
                junction,
                status
            }}
        }}
    """
    return gql(gqlString)


# ************** Users **************

def addUser(username, email):
    gqlString = f"""
        mutation AddUser {{
            createUser(data: {{
                username: "{username}"
                email: "{email}"
            }}) {{
                _id,
                username,
                email
            }}
        }}
    """
    return gql(gqlString)

def getUsers():
    return gql(
    """
        query GetUsers {
            allUsers {
                data {
                    _id,
                    username,
                    email
                }
            }
        }
    """
    )

def updateUser(id, username, email):
    gqlString = f"""
        mutation UpdateUser {{
            updateUser(id: {id}, data: {{
                username: "{username}"
                email: "{email}"
            }}) {{
                _id,
                username,
                email
            }}
        }}
    """
    return gql(gqlString)

def deleteUser(id):
    gqlString = f"""
    mutation DeleteUser {{
        deleteUser(id: {id}) {{
            _id
        }}
    }}
    """
    return gql(gqlString)

# ************** Map **************

# getMap = gql()


def addTestRequest():
    result = executeAPI(addRequest("1","AB","CD","G","F","pending"))
    return result['createRequest']['_id']

def addAndDeleteRequest():
    start = executeAPI(getRequests())
    executeAPI(deleteRequest(addTestRequest()))
    end = executeAPI(getRequests())

    if (start == end):
        return True
    else:
        return False

def updateAndRevertRequest():
    id = addTestRequest()
    executeAPI(updateRequest(id,8,"1","AB","CD","G","F","pending"))
    start = executeAPI(getRequests())
    executeAPI(updateRequest(id,7,"1","LO","GH","A","C","completed"))
    mid = executeAPI(getRequests())
    executeAPI(updateRequest(id,8,"1","AB","CD","G","F","pending"))
    end = executeAPI(getRequests())
    executeAPI(deleteRequest(id))
    executeAPI(getRequests())

    if (start == end and start != mid):
        return True
    else:
        return False


def addTestRobot():
    result = executeAPI(addRobot("InService"))
    return result['createRobot']['_id']

def addAndDeleteRobot():
    start = executeAPI(getRobots())
    executeAPI(deleteRobot(addTestRobot()))
    end = executeAPI(getRobots())

    if (start == end):
        return True
    else:
        return False

def updateAndRevertRobot():
    id = addTestRobot()
    executeAPI(updateRobot(id,"a","InService"))
    start = executeAPI(getRobots())
    executeAPI(updateRobot(id,"b","Charging"))
    mid = executeAPI(getRobots())
    executeAPI(updateRobot(id,"a","InService"))
    end = executeAPI(getRobots())
    executeAPI(deleteRobot(id))
    executeAPI(getRobots())

    if (start == end and start != mid):
        return True
    else:
        return False


def addTestJunction():
    result = executeAPI(addJunction())
    return result['createJunction']['_id']

def addAndDeleteJunction():
    start = executeAPI(getJunctions())
    executeAPI(deleteJunction(addTestJunction()))
    end = executeAPI(getJunctions())

    if (start == end):
        return True
    else:
        return False

def updateAndRevertJunction():
    id = addTestJunction()
    executeAPI(updateJunction(id,[214123213]))
    start = executeAPI(getJunctions())
    executeAPI(updateJunction(id,[214123213,31837]))
    mid = executeAPI(getJunctions())
    executeAPI(updateJunction(id,[214123213]))
    end = executeAPI(getJunctions())
    executeAPI(deleteJunction(id))
    executeAPI(getJunctions())

    if (start == end and start != mid):
        return True
    else:
        return False


def addTestBeacon():
    result = executeAPI(addBeacon("InService"))
    return result['createBeacon']['_id']

def addAndDeleteBeacon():
    start = executeAPI(getBeacons())
    executeAPI(deleteBeacon(addTestBeacon()))
    end = executeAPI(getBeacons())

    if (start == end):
        return True
    else:
        return False

def updateAndRevertBeacon():
    id = addTestBeacon()
    executeAPI(updateBeacon(id,214123213,"InService"))
    start = executeAPI(getBeacons())
    executeAPI(updateBeacon(id,3142212,"BeaconFault"))
    mid = executeAPI(getBeacons())
    executeAPI(updateBeacon(id,214123213,"InService"))
    end = executeAPI(getBeacons())
    executeAPI(deleteBeacon(id))
    executeAPI(getBeacons())

    if (start == end and start != mid):
        return True
    else:
        return False


def addTestUser():
    result = executeAPI(addUser("test12334","test12334@test.com"))
    return result['createUser']['_id']

def addAndDeleteUser():
    start = executeAPI(getUsers())
    executeAPI(deleteUser(addTestUser()))
    end = executeAPI(getUsers())

    if (start == end):
        return True
    else:
        return False

def updateAndRevertUser():
    id = addTestUser()
    start = executeAPI(getUsers())
    executeAPI(updateUser(id,"test12334","wdehuwdwhuduwh@bob.com"))
    mid = executeAPI(getUsers())
    executeAPI(updateUser(id,"test12334","test12334@test.com"))
    end = executeAPI(getUsers())
    executeAPI(deleteUser(id))
    executeAPI(getUsers())

    if (start == end and start != mid):
        return True
    else:
        return False


def runTests():
    allTestsPassed = True
    if (not addAndDeleteRequest()):
        print ("FAIL: addAndDeleteRequest")
        allTestsPassed = False
    if (not updateAndRevertRequest()):
        print ("FAIL: updateAndRevertRequest")
        allTestsPassed = False

    if (not addAndDeleteRobot()):
        print ("FAIL: addAndDeleteRobot")
        allTestsPassed = False
    if (not updateAndRevertRobot()):
        print ("FAIL: updateAndRevertRobot")
        allTestsPassed = False

    if (not addAndDeleteJunction()):
        print ("FAIL: addAndDeleteJunction")
        allTestsPassed = False
    if (not updateAndRevertJunction()):
        print ("FAIL: updateAndRevertJunction")
        allTestsPassed = False

    if (not addAndDeleteBeacon()):
        print ("FAIL: addAndDeleteBeacon")
        allTestsPassed = False
    if (not updateAndRevertBeacon()):
        print ("FAIL: updateAndRevertBeacon")
        allTestsPassed = False

    if (not addAndDeleteUser()):
        print ("FAIL: addAndDeleteUser")
        allTestsPassed = False
    if (not updateAndRevertUser()):
        print ("FAIL: updateAndRevertUser")
        allTestsPassed = False

    if (allTestsPassed):
        print ("PASS: All tests")

runTests()

