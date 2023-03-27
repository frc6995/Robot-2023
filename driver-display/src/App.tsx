import React, { Component } from 'react';
import logo from './logo.svg';
import './App.css';
import {NetworkTables} from 'ntcore-ts-client';
import { NetworkTableTypeInfos } from 'ntcore-ts-client/src/lib/types/types';





type Props = {selection:number;}

type RowProps = {selection:number; row:number}

function Row(props:RowProps) {
  return <div style={{flexGrow:1, display:'flex', flexDirection:'row-reverse'}} >
    {(()=>{
      let content = [];
      for (let i = 0; i < 9; i++) {
        var id = props.row * 9 + i;
        var backgroundColor = '';
        if (props.selection == id) {
          backgroundColor = 'green';
        }
        else if (props.row == 0) {
          backgroundColor = 'lightgray';
        }
        else  if (i % 3 == 1) {
          backgroundColor = 'purple';
        }
        else {
          backgroundColor = 'yellow';
        }
        content.push(
          <div key={id} style={{
            background: backgroundColor,
            height:"100%",
            flexGrow:1,
            border:'2px solid black'

          }}>
            </div>
        )
        
      }
      return content;
    })()}
  </div>
}

function Grid(props:Props) {
  return (
    <div style={{width:"100vw", height:"33vw", display:'flex', flexDirection:'column', justifyContent:"space-evenly"}}>
      <Row row={0} selection={props.selection}></Row>
      <Row row={1} selection={props.selection}></Row>
      <Row row={2} selection={props.selection}></Row>
    </div>
  )
}
  
type AppState = {selection:number, time:number, autoSelection: string, autoOptions: string[], connected:boolean};

class App extends Component{
  state: AppState = {selection:-1, time:0, autoSelection: '', autoOptions: [''], connected: false};
  ntcore = NetworkTables.createInstanceByURI("10.69.95.2");
  selectionSubscriberUID=0;
  matchTimeUID = 0;
  autoActiveUID = 0;
  autoOptionsUID = 0;
  removeConnectionListener = ()=>{};

  selectionTopic = this.ntcore.createTopic<number>('/DriverDisplay/selection', NetworkTableTypeInfos.kInteger);
  matchTimeTopic = this.ntcore.createTopic<number>('/DriverDisplay/matchTime', NetworkTableTypeInfos.kDouble);
  autoSelectedTopic = this.ntcore.createTopic<string>('/SmartDashboard/SendableChooser[0]/selected', NetworkTableTypeInfos.kString);
  autoActiveTopic = this.ntcore.createTopic<string>('/SmartDashboard/SendableChooser[0]/active', NetworkTableTypeInfos.kString);

  autoOptionsTopic = this.ntcore.createTopic<string[]>('/SmartDashboard/SendableChooser[0]/options', NetworkTableTypeInfos.kStringArray);
  
  constructor(props: any) {
    super(props);
    this.selectionSubscriberUID = this.selectionTopic!.subscribe((value)=>{this.setState({selection: (value ===null ? -1 : value)}); console.log(value); });
    this.matchTimeUID = this.matchTimeTopic!.subscribe((value)=>{
      this.setState({time: (value ===null ? -1 : Math.ceil(value))});});
    this.autoActiveUID = this.autoActiveTopic!.subscribe((value)=>this.setState({autoSelection: value === null ? 'Disconnected' : value}))


    this.autoSelectedTopic!.publish({retained:true});
    this.autoOptionsUID = this.autoOptionsTopic!.subscribe((value)=>this.setState({autoOptions: value === null ? '' : value}))
    this.state.autoOptions = this.autoOptionsTopic.getValue() || ['Disconnected'];
    this.removeConnectionListener = this.ntcore.addRobotConnectionListener((connected)=>this.state.connected = connected, true);
    //console.log(this.selectionSubscriberUID)
  }
  
  componentWillUnmount(): void {
    this.selectionTopic.unsubscribe(this.selectionSubscriberUID, true);
    this.matchTimeTopic.unsubscribe(this.matchTimeUID, true);
    this.autoActiveTopic.unsubscribe(this.autoActiveUID, true);
    this.autoSelectedTopic.unpublish();
    this.autoOptionsTopic.unsubscribe(this.autoOptionsUID, true);
    this.removeConnectionListener();
  }
  render() {
    // /console.log(this.state.autoOptions);
    return (
      <div className="App">
        <Grid selection={this.state.selection}></Grid>


        <div style={{color:"white", fontSize: "10vw", display:"flex", justifyContent:"space-evenly"}}>
          <select style={{width:"45%"}} value={this.state.autoSelection} onChange={(event)=>this.autoSelectedTopic.setValue(event.target.value)}>
            {this.state.autoOptions.map((value)=>(<option value={value} key={value}>{value}</option>))}
          </select>
        <div style={{width:"10%", display:"flex", flexDirection:"column"}}>
          <button style={{width:"100%", height:"50%"}} onClick={()=>window.open("http://10.69.95.11:1182/stream.mjpg")}>Camera</button>
          <span style={{width:"100%", height:"50%", backgroundColor:this.state.connected ? "green" : "red"}}></span>
        </div>
        <span style={{width:"45%"}}>
          {this.state.time}
        </span>
          {/*{this.state.time/*this.state.time === -1 ? "--": `${Math.floor(this.state.time / 60).toFixed(0)}:${(this.state.time % 60).toString().padStart(2, '0')}`*/}
        </div>
      </div>
    );
    }

}

export default App;
