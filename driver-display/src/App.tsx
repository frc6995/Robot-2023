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
          <div style={{
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
  
type AppState = {selection:number};

class App extends Component{
  state: AppState = {selection:-1};
  ntcore = NetworkTables.createInstanceByURI("10.69.95.2");
  selectionSubscriberUID=0;

  selectionTopic = this.ntcore.createTopic<number>('/DriverDisplay/selection', NetworkTableTypeInfos.kInteger);
  
  constructor(props: any) {
    super(props);
    this.selectionSubscriberUID = this.selectionTopic!.subscribe((value)=>{this.setState({selection: (value ===null ? -1 : value)}); console.log(value); });
    console.log(this.selectionSubscriberUID)
  }
  
  componentWillUnmount(): void {
    this.selectionTopic.unsubscribe(this.selectionSubscriberUID, true);
  }
  render() {
    return (
      <div className="App">
        <Grid selection={this.state.selection}></Grid>
      </div>
    );
    }

}

export default App;
