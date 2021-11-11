export class MailRequest {

  constructor(
    public id: number,
    public timestamp: Date,
    public source: string,
    public destination: string,
    public sender: string,
    public recipient: string
    ) {  }


    toString(): string {
      return '{"id":"' + this.id + '","timestamp":"' + this.timestamp.toISOString() + '","source":"' + this.source + '","destination":"' + this.destination + '","sender":"' + this.sender + '", "recipient":"' + this.recipient + '"}';
    }
}