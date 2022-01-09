export class MailRequest {

  constructor(
    public id: number,
    public robot: number,
    public timestamp: Date,
    public source: string,
    public destination: string,
    public sender: string,
    public recipient: string,
    public state: string
    ) {  }


    toString(): string {
      return '{"id":"' + this.id + '","robot":"' + this.robot + '","timestamp":"' + this.timestamp.toISOString() + '","source":"' + this.source + '","destination":"' + this.destination + '","sender":"' + this.sender + '", "recipient":"' + this.recipient + '","state":"' + this.state +  '"}';
    }
}