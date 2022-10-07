function sysCall_init()
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function sysCall_cleanup()
    -- do some clean-up here
end

function coroutineMain()
    -- Put some initialization code here
    i=true
    lineSize=3 -- in points
    maximumLines=9999
    red={1,0,0}
    drawingContainer=sim.addDrawingObject(sim.drawing_points,lineSize,0,-1,maximumLines,red) -- adds a line
    -- Put your main loop here, e.g.:
    --
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        if i then
            print("Iniciando Caminho")
            i = false
        end
        myData=sim.getStringSignal("point_coord")
        sim.clearStringSignal('point_coord')
        if (myData) then
            data=sim.unpackFloatTable(myData)
            data[#data+1]=0.01
            sim.addDrawingObjectItem(drawingContainer,data)
            print(data)
        end
       sim.switchThread() -- resume in next simulation step
    end

    -- Put your main loop here, e.g.:
    --
    -- while true do
    --     local p=sim.getObjectPosition(objHandle,-1)
    --     p[1]=p[1]+0.001
    --     sim.setObjectPosition(objHandle,-1,p)
    --     sim.switchThread() -- resume in next simulation step
    -- end
end

-- See the user manual or the available code snippets for additional callback functions and details
