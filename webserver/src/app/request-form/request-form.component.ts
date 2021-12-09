import { Component, OnInit } from '@angular/core';
import { MailRequest } from '../mail-request';
import { MailRequestService } from '../mail-request.service';
import { MessageService } from '../message.service';

@Component({
  selector: 'app-request-form',
  templateUrl: './request-form.component.html',
  styleUrls: ['./request-form.component.css']
})
export class RequestFormComponent implements OnInit {

  locations = ['UC', 'MC', 'CB', 'ME'];

  model = new MailRequest(1005, new Date(0), 'UC', 'MC', 'Alice', 'Bob');

  id: number = -1;

  submitted = false;

  constructor(private mailRequestService: MailRequestService, private messageService: MessageService) { }

  ngOnInit(): void {
    this.mailRequestService.getNextID().subscribe(id => this.id = id);
  }

  onSubmit() { this.submitted = true; }

  newRequest() {
    this.model = new MailRequest(-1, new Date(0), 'UC', 'MC', 'Alice', 'Bob');
  }

  submitRequest() {
    this.model.id = this.id;
    this.model.timestamp = new Date(Date.now());
    this.messageService.add('RequestFormComponent: new request submitted ' + this.model.toString());
    this.id++;
  }

}
