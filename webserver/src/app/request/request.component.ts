import { Component, OnInit } from '@angular/core';
import { MailRequest } from '../mail-request';
import { MailRequestService } from '../mail-request.service';
import { MessageService } from '../message.service';

@Component({
  selector: 'app-request',
  templateUrl: './request.component.html',
  styleUrls: ['./request.component.css']
})
export class RequestComponent implements OnInit {

  requests: MailRequest[] = [];

  constructor(private mailRequestService: MailRequestService, private messageService: MessageService) { 
    
  }

  ngOnInit(): void {
    this.getRequests();
  }

  getRequests(): void {
    this.mailRequestService.getRequests().subscribe(requests => this.requests = requests);
  }

}
